//
//  Renderer.swift
//  GrootPlayer
//
//  Created by Kyungjin Lee on 2020/06/10.
//  Copyright © 2020 Kyungjin Lee. All rights reserved.
//

import Foundation
import Metal
import MetalKit
import ARKit

protocol RenderDestinationProvider {
    var currentRenderPassDescriptor: MTLRenderPassDescriptor? { get }
    var currentDrawable: CAMetalDrawable? { get }
    var colorPixelFormat: MTLPixelFormat { get set }
    var depthStencilPixelFormat: MTLPixelFormat { get set }
    var sampleCount: Int { get set }
}

// The max number of command buffers in flight
let kMaxBuffersInFlight: Int = 3

// The max number anchors our uniform buffer will hold
let kMaxAnchorInstanceCount: Int = 64

// The 16 byte aligned size of our uniform structures
let kAlignedSharedUniformsSize: Int = (MemoryLayout<SharedUniforms>.size & ~0xFF) + 0x100
let kAlignedInstanceUniformsSize: Int = ((MemoryLayout<InstanceUniforms>.size * kMaxAnchorInstanceCount) & ~0xFF) + 0x100

// Vertex data for an image plane
let kImagePlaneVertexData: [Float] = [
    -1.0, -1.0,  0.0, 1.0,
    1.0, -1.0,  1.0, 1.0,
    -1.0,  1.0,  0.0, 0.0,
    1.0,  1.0,  1.0, 0.0,
]


class Renderer {
    let session: ARSession
    let device: MTLDevice
    let inFlightSemaphore = DispatchSemaphore(value: kMaxBuffersInFlight)
    var renderDestination: RenderDestinationProvider
    
    // Session params
    var isSaveViewTrace: Bool = false
    var fileList: [String] = []
    var isFinishVideo: Bool = false
    var isFinishSave: Bool = false
    var isUpdateFrame: Bool = false
    var cntRenderFrame = 0
    var manifest_ = UnsafeMutablePointer<Manifest>.allocate(capacity:1)
    var lastRenderTime =  Date()
    var currentRenderTime = Date()
    var rootTransform: Float = 0.0
    
    
    
    
    // for write view trace file
    var testCntViewTrace = 0
    var testCntUpdateFrame = 0
    var trackViewMat = matrix_float4x4()
    var trackProjMat = matrix_float4x4()
    var writeViewTraceFileURL: URL!
    var viewTraceFileBufferPointer: UnsafeMutableBufferPointer<matrix_float4x4>!
    var viewTraceCount: Int = 0
    var writeViewTraceArray: [matrix_float4x4] = []
    
    // Point cloud frame
    var pointCloudVertexData: [UInt8] = []
    var pointCloudColorData: [UInt8] = []
    
    // Point cloud buffer
    
    var currentRenderFrame = UnsafeMutablePointer<RenderFrame>.allocate(capacity: 1)
    var renderFrameQueue = Queue<UnsafeMutablePointer<RenderFrame>>()
    
    var renderFrameCenterBuffer: MTLBuffer!
    var renderFrameDepthBuffer: MTLBuffer!
    var renderFrameColorBuffer: MTLBuffer!
    
    var pointCloudPipelineState: MTLRenderPipelineState!
    var pointCloudDepthState: MTLDepthStencilState!
    var pointCloudVertexDescriptor: MTLVertexDescriptor!
    
    
    
    // Metal objects
    var commandQueue: MTLCommandQueue!
    var sharedUniformBuffer: MTLBuffer!
    var anchorUniformBuffer: MTLBuffer!
    var imagePlaneVertexBuffer: MTLBuffer!
    var capturedImagePipelineState: MTLRenderPipelineState!
    var capturedImageDepthState: MTLDepthStencilState!
    var anchorPipelineState: MTLRenderPipelineState!
    var anchorDepthState: MTLDepthStencilState!
    var capturedImageTextureY: CVMetalTexture?
    var capturedImageTextureCbCr: CVMetalTexture?
    
    // Captured image texture cache
    var capturedImageTextureCache: CVMetalTextureCache!
    
    // Metal vertex descriptor specifying how vertices will by laid out for input into our
    //   anchor geometry render pipeline and how we'll layout our Model IO verticies
    var geometryVertexDescriptor: MTLVertexDescriptor!
    
    // MetalKit mesh containing vertex data and index buffer for our anchor geometry
    var cubeMesh: MTKMesh!
    
    // Used to determine _uniformBufferStride each frame.
    //   This is the current frame number modulo kMaxBuffersInFlight
    var uniformBufferIndex: Int = 0
    
    // Offset within _sharedUniformBuffer to set for the current frame
    var sharedUniformBufferOffset: Int = 0
    
    // Offset within _anchorUniformBuffer to set for the current frame
    var anchorUniformBufferOffset: Int = 0
    
    // Addresses to write shared uniforms to each frame
    var sharedUniformBufferAddress: UnsafeMutableRawPointer!
    
    // Addresses to write anchor uniforms to each frame
    var anchorUniformBufferAddress: UnsafeMutableRawPointer!
    
    // The number of anchor instances to render
    var anchorInstanceCount: Int = 0
    
    // The current viewport size
    var viewportSize: CGSize = CGSize()
    
    // Flag for viewport size changes
    var viewportSizeDidChange: Bool = false
    
    let fileManager = FileManager.default
    
    
    init(session: ARSession, metalDevice device: MTLDevice, renderDestination: RenderDestinationProvider, userName: String, isSaveViewTrace: Bool, fileList: [String], initialDepth: Float) {
        self.session = session
        self.device = device
        self.renderDestination = renderDestination
        self.rootTransform = initialDepth
        
        
        self.writeViewTraceFileURL = FileManager.documentsDirectoryURL.appendingPathComponent(userName + "_viewtrace").appendingPathExtension("txt")
        
        
        
        self.fileList = fileList
        self.isSaveViewTrace = isSaveViewTrace
        
        loadMetal()
        receive_manifest(manifest_ );
        
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else {
                return
            }
            self.loadFrames()
        }
    }
    
    func loadFrames(){
        var frameCnt = 0
        while(frameCnt < fileList.count)
        {
            // To ensure 30fps when decoding is too fast
            currentRenderTime = Date()
            if(currentRenderTime.timeIntervalSince(lastRenderTime) < 0.033){
                let diff = 0.033 - currentRenderTime.timeIntervalSince(lastRenderTime)
                usleep((useconds_t)(diff * 1000000))
            }
            lastRenderTime = currentRenderTime
            
            // Set maximum queue count to 100
            if(renderFrameQueue.count < 100)
            {
                autoreleasepool{
                    var renderFrame = UnsafeMutablePointer<RenderFrame>.allocate(capacity: 1)
                    let readFilePath = Bundle.main.path(forResource: fileList[frameCnt], ofType: "bin")!
                    cReadFrameFromFile(readFilePath.cString(using: .utf8), rootTransform, renderFrame)
                    renderFrameQueue.enqueue(renderFrame)
                    
                    
                    frameCnt = frameCnt + 1
                }
            }
        }
        
    }
    
    
    func writeViewTrace(){
        let viewTraceData = Data(bytes: &writeViewTraceArray, count: writeViewTraceArray.count * MemoryLayout<matrix_float4x4>.stride)
        try! viewTraceData.write(to: writeViewTraceFileURL)
    }
    
    func drawRectResized(size: CGSize) {
        viewportSize = size
        viewportSizeDidChange = true
    }
    
    func update() {
        // Wait to ensure only kMaxBuffersInFlight are getting proccessed by any stage in the Metal
        //   pipeline (App, Metal, Drivers, GPU, etc)
        let _ = inFlightSemaphore.wait(timeout: DispatchTime.distantFuture)
        
        // Create a new command buffer for each renderpass to the current drawable
        if let commandBuffer = commandQueue.makeCommandBuffer() {
            commandBuffer.label = "MyCommand"
            
            // Add completion hander which signal _inFlightSemaphore when Metal and the GPU has fully
            //   finished proccssing the commands we're encoding this frame.  This indicates when the
            //   dynamic buffers, that we're writing to this frame, will no longer be needed by Metal
            //   and the GPU.
            // Retain our CVMetalTextures for the duration of the rendering cycle. The MTLTextures
            //   we use from the CVMetalTextures are not valid unless their parent CVMetalTextures
            //   are retained. Since we may release our CVMetalTexture ivars during the rendering
            //   cycle, we must retain them separately here.
            var textures = [capturedImageTextureY, capturedImageTextureCbCr]
            commandBuffer.addCompletedHandler{ [weak self] commandBuffer in
                if let strongSelf = self {
                    strongSelf.inFlightSemaphore.signal()
                }
                textures.removeAll()
            }
            
            updateBufferStates()
            updateGameState()
            
            if let renderPassDescriptor = renderDestination.currentRenderPassDescriptor, let currentDrawable = renderDestination.currentDrawable, let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) {
                
                renderEncoder.label = "MyRenderEncoder"
                
                drawCapturedImage(renderEncoder: renderEncoder)
                drawPointCloudGeometry(renderEncoder: renderEncoder)
                
                // We're done encoding commands
                renderEncoder.endEncoding()
                
                // Schedule a present once the framebuffer is complete using the current drawable
                commandBuffer.present(currentDrawable)
            }
            
            // Finalize rendering here & push the command buffer to the GPU
            commandBuffer.commit()
        }
    }
    
    // MARK: - Private
    func loadMetal() {
        // Create and load our basic Metal state objects
        
        // Set the default formats needed to render
        renderDestination.depthStencilPixelFormat = .depth32Float_stencil8
        renderDestination.colorPixelFormat = .bgra8Unorm
        renderDestination.sampleCount = 1
        
        // Calculate our uniform buffer sizes. We allocate kMaxBuffersInFlight instances for uniform
        //   storage in a single buffer. This allows us to update uniforms in a ring (i.e. triple
        //   buffer the uniforms) so that the GPU reads from one slot in the ring wil the CPU writes
        //   to another. Anchor uniforms should be specified with a max instance count for instancing.
        //   Also uniform storage must be aligned (to 256 bytes) to meet the requirements to be an
        //   argument in the constant address space of our shading functions.
        let sharedUniformBufferSize = kAlignedSharedUniformsSize * kMaxBuffersInFlight
        let anchorUniformBufferSize = kAlignedInstanceUniformsSize * kMaxBuffersInFlight
        
        // Create and allocate our uniform buffer objects. Indicate shared storage so that both the
        //   CPU can access the buffer
        sharedUniformBuffer = device.makeBuffer(length: sharedUniformBufferSize, options: .storageModeShared)
        sharedUniformBuffer.label = "SharedUniformBuffer"
        
        anchorUniformBuffer = device.makeBuffer(length: anchorUniformBufferSize, options: .storageModeShared)
        anchorUniformBuffer.label = "AnchorUniformBuffer"
        
        // Create a vertex buffer with our image plane vertex data.
        let imagePlaneVertexDataCount = kImagePlaneVertexData.count * MemoryLayout<Float>.size
        imagePlaneVertexBuffer = device.makeBuffer(bytes: kImagePlaneVertexData, length: imagePlaneVertexDataCount, options: [])
        imagePlaneVertexBuffer.label = "ImagePlaneVertexBuffer"
        
        // Load all the shader files with a metal file extension in the project
        let defaultLibrary = device.makeDefaultLibrary()!
        
        let capturedImageVertexFunction = defaultLibrary.makeFunction(name: "capturedImageVertexTransform")!
        let capturedImageFragmentFunction = defaultLibrary.makeFunction(name: "capturedImageFragmentShader")!
        
        // Create a vertex descriptor for our image plane vertex buffer
        let imagePlaneVertexDescriptor = MTLVertexDescriptor()
        
        // Positions.
        imagePlaneVertexDescriptor.attributes[0].format = .float2
        imagePlaneVertexDescriptor.attributes[0].offset = 0
        imagePlaneVertexDescriptor.attributes[0].bufferIndex = Int(kBufferIndexMeshPositions.rawValue)
        
        // Texture coordinates.
        imagePlaneVertexDescriptor.attributes[1].format = .float2
        imagePlaneVertexDescriptor.attributes[1].offset = 8
        imagePlaneVertexDescriptor.attributes[1].bufferIndex = Int(kBufferIndexMeshPositions.rawValue)
        
        // Buffer Layout
        imagePlaneVertexDescriptor.layouts[0].stride = 16
        imagePlaneVertexDescriptor.layouts[0].stepRate = 1
        imagePlaneVertexDescriptor.layouts[0].stepFunction = .perVertex
        
        // Create a pipeline state for rendering the captured image
        let capturedImagePipelineStateDescriptor = MTLRenderPipelineDescriptor()
        capturedImagePipelineStateDescriptor.label = "MyCapturedImagePipeline"
        capturedImagePipelineStateDescriptor.sampleCount = renderDestination.sampleCount
        capturedImagePipelineStateDescriptor.vertexFunction = capturedImageVertexFunction
        capturedImagePipelineStateDescriptor.fragmentFunction = capturedImageFragmentFunction
        capturedImagePipelineStateDescriptor.vertexDescriptor = imagePlaneVertexDescriptor
        capturedImagePipelineStateDescriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        capturedImagePipelineStateDescriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        capturedImagePipelineStateDescriptor.stencilAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        
        do {
            try capturedImagePipelineState = device.makeRenderPipelineState(descriptor: capturedImagePipelineStateDescriptor)
        } catch let error {
            print("Failed to created captured image pipeline state, error \(error)")
        }
        
        let capturedImageDepthStateDescriptor = MTLDepthStencilDescriptor()
        capturedImageDepthStateDescriptor.depthCompareFunction = .always
        capturedImageDepthStateDescriptor.isDepthWriteEnabled = false
        capturedImageDepthState = device.makeDepthStencilState(descriptor: capturedImageDepthStateDescriptor)
        
        // Create captured image texture cache
        var textureCache: CVMetalTextureCache?
        CVMetalTextureCacheCreate(nil, nil, device, nil, &textureCache)
        capturedImageTextureCache = textureCache
        
        // Create a vertex descriptor for our Metal pipeline. Specifies the layout of vertices the
        //   pipeline should expect. The layout below keeps attributes used to calculate vertex shader
        //   output position separate (world position, skinning, tweening weights) separate from other
        //   attributes (texture coordinates, normals).  This generally maximizes pipeline efficiency
        let pointCloudVertexFunction = defaultLibrary.makeFunction(name:"pointcloud_vertex_shader")!
        let pointCloudFragmentFunction = defaultLibrary.makeFunction(name:"pointcloud_fragment_shader")!
        
        
        // Octree Breadth Bytes
        pointCloudVertexDescriptor = MTLVertexDescriptor()
        pointCloudVertexDescriptor.attributes[0].format = .float4
        pointCloudVertexDescriptor.attributes[0].offset = 0
        pointCloudVertexDescriptor.attributes[0].bufferIndex = 0
        
        // Octree Depth Bytes
        pointCloudVertexDescriptor.attributes[1].format = .uchar4
        pointCloudVertexDescriptor.attributes[1].offset = 0
        pointCloudVertexDescriptor.attributes[1].bufferIndex = 1
        
        // Octree Color Bytes
        pointCloudVertexDescriptor.attributes[2].format = .uchar4
        pointCloudVertexDescriptor.attributes[2].offset = 0
        pointCloudVertexDescriptor.attributes[2].bufferIndex = 2
        
        pointCloudVertexDescriptor.layouts[0].stride = MemoryLayout<float4>.stride
        pointCloudVertexDescriptor.layouts[1].stride = MemoryLayout<uint>.stride
        pointCloudVertexDescriptor.layouts[2].stride = MemoryLayout<uint>.stride
        
        let pointCloudPipelineStateDescriptor = MTLRenderPipelineDescriptor()
        pointCloudPipelineStateDescriptor.label = "IFramePipeline"
        pointCloudPipelineStateDescriptor.sampleCount = renderDestination.sampleCount
        pointCloudPipelineStateDescriptor.vertexFunction = pointCloudVertexFunction
        pointCloudPipelineStateDescriptor.fragmentFunction = pointCloudFragmentFunction
        pointCloudPipelineStateDescriptor.vertexDescriptor = pointCloudVertexDescriptor
        pointCloudPipelineStateDescriptor.colorAttachments[0].pixelFormat = renderDestination.colorPixelFormat
        pointCloudPipelineStateDescriptor.depthAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        pointCloudPipelineStateDescriptor.stencilAttachmentPixelFormat = renderDestination.depthStencilPixelFormat
        
        do {
            try pointCloudPipelineState = device.makeRenderPipelineState(descriptor: pointCloudPipelineStateDescriptor)
        } catch let error {
            print("Failed to create iframe geometry pipeline state, error \(error)")
        }
        
        let pointCloudDepthStateDescriptor = MTLDepthStencilDescriptor()
        pointCloudDepthStateDescriptor.depthCompareFunction = .less
        pointCloudDepthStateDescriptor.isDepthWriteEnabled = true
        pointCloudDepthState = device.makeDepthStencilState(descriptor: pointCloudDepthStateDescriptor)
        
        
        
        
        // Create the command queue
        commandQueue = device.makeCommandQueue()
    }
    
    func updateBufferStates() {
        // Update the location(s) to which we'll write to in our dynamically changing Metal buffers for
        //   the current frame (i.e. update our slot in the ring buffer used for the current frame)
        
        uniformBufferIndex = (uniformBufferIndex + 1) % kMaxBuffersInFlight
        
        sharedUniformBufferOffset = kAlignedSharedUniformsSize * uniformBufferIndex
        anchorUniformBufferOffset = kAlignedInstanceUniformsSize * uniformBufferIndex
        
        sharedUniformBufferAddress = sharedUniformBuffer.contents().advanced(by: sharedUniformBufferOffset)
        anchorUniformBufferAddress = anchorUniformBuffer.contents().advanced(by: anchorUniformBufferOffset)
    }
    
    func updateGameState() {
        // Update any game state
        
        guard let currentFrame = session.currentFrame else {
            return
        }
        
        updateSharedUniforms(frame: currentFrame)
        
        if(renderFrameQueue.isEmpty)
        {
            self.isFinishVideo = true
            /*if(self.isSaveViewTrace && !self.isFinishSave){
             writeViewTrace()
             self.isFinishSave = true
             }*/
        }
        else
        {
            print("PointCloudFrameBuffer: ", renderFrameQueue.count)
            // To update in 30fps because draw calls in 60fps
            if(isUpdateFrame){
                updateFrame()
                isUpdateFrame = false
            }else{
                isUpdateFrame = true
            }
        }
        
        
        updateAnchors(frame: currentFrame)
        updateCapturedImageTextures(frame: currentFrame)
        
        if viewportSizeDidChange {
            viewportSizeDidChange = false
            
            updateImagePlane(frame: currentFrame)
        }
    }
    
    
    func updateSharedUniforms(frame: ARFrame) {
        // Update the shared uniforms of the frame
        
        let uniforms = sharedUniformBufferAddress.assumingMemoryBound(to: SharedUniforms.self)
        
        uniforms.pointee.viewMatrix = frame.camera.viewMatrix(for: .landscapeRight)
        uniforms.pointee.projectionMatrix = frame.camera.projectionMatrix(for: .landscapeRight, viewportSize: viewportSize, zNear: 0.001, zFar: 1000)
        
        trackViewMat = uniforms.pointee.viewMatrix
        trackProjMat = uniforms.pointee.projectionMatrix
        
        
        // Set up lighting for the scene using the ambient intensity if provided
        
        var ambientIntensity: Float = 1.0
        
        if let lightEstimate = frame.lightEstimate {
            ambientIntensity = Float(lightEstimate.ambientIntensity) / 1000.0
        }
        
        let ambientLightColor: vector_float3 = vector3(0.5, 0.5, 0.5)
        uniforms.pointee.ambientLightColor = ambientLightColor * ambientIntensity
        
        var directionalLightDirection : vector_float3 = vector3(0.0, 0.0, -1.0)
        directionalLightDirection = simd_normalize(directionalLightDirection)
        uniforms.pointee.directionalLightDirection = directionalLightDirection
        
        let directionalLightColor: vector_float3 = vector3(0.6, 0.6, 0.6)
        uniforms.pointee.directionalLightColor = directionalLightColor * ambientIntensity
        
        uniforms.pointee.materialShininess = 30
        uniforms.pointee.sidelength = manifest_.pointee.max_breadth_depth_sidelength
        
    }
    
    func updateFrame(){
        
        if(cntRenderFrame > 0){
            currentRenderFrame.pointee.center_list.deallocate()
            currentRenderFrame.pointee.color_bytes.deallocate()
            currentRenderFrame.pointee.depth_bytes.deallocate()
        }
        currentRenderFrame = renderFrameQueue.dequeue()!
        testCntUpdateFrame = testCntUpdateFrame + 1
        writeViewTraceArray.append(trackViewMat)
        writeViewTraceArray.append(trackProjMat)
        
        testCntViewTrace = testCntViewTrace + 1
        cntRenderFrame = cntRenderFrame + 1
        
        
        if(cntRenderFrame == fileList.count )
        {
            
            if(self.isSaveViewTrace && !self.isFinishSave){
                writeViewTrace()
                self.isFinishSave = true
            }
        }
        
    }
    
    func updateAnchors(frame: ARFrame) {
        // Update the anchor uniform buffer with transforms of the current frame's anchors
        anchorInstanceCount = min(frame.anchors.count, kMaxAnchorInstanceCount)
        
        var anchorOffset: Int = 0
        if anchorInstanceCount == kMaxAnchorInstanceCount {
            anchorOffset = max(frame.anchors.count - kMaxAnchorInstanceCount, 0)
        }
        
        for index in 0..<anchorInstanceCount {
            let anchor = frame.anchors[index + anchorOffset]
            
            // Flip Z axis to convert geometry from right handed to left handed
            var coordinateSpaceTransform = matrix_identity_float4x4
            //coordinateSpaceTransform.columns.2.z = -1.0
            
            let modelMatrix = simd_mul(anchor.transform, coordinateSpaceTransform)
            
            let anchorUniforms = anchorUniformBufferAddress.assumingMemoryBound(to: InstanceUniforms.self).advanced(by: index)
            anchorUniforms.pointee.modelMatrix = modelMatrix
        }
    }
    
    // Update image texture with camera frame
    func updateCapturedImageTextures(frame: ARFrame) {
        // Create two textures (Y and CbCr) from the provided frame's captured image
        let pixelBuffer = frame.capturedImage
        
        if (CVPixelBufferGetPlaneCount(pixelBuffer) < 2) {
            return
        }
        
        capturedImageTextureY = createTexture(fromPixelBuffer: pixelBuffer, pixelFormat:.r8Unorm, planeIndex:0)
        capturedImageTextureCbCr = createTexture(fromPixelBuffer: pixelBuffer, pixelFormat:.rg8Unorm, planeIndex:1)
    }
    
    func createTexture(fromPixelBuffer pixelBuffer: CVPixelBuffer, pixelFormat: MTLPixelFormat, planeIndex: Int) -> CVMetalTexture? {
        let width = CVPixelBufferGetWidthOfPlane(pixelBuffer, planeIndex)
        let height = CVPixelBufferGetHeightOfPlane(pixelBuffer, planeIndex)
        
        var texture: CVMetalTexture? = nil
        let status = CVMetalTextureCacheCreateTextureFromImage(nil, capturedImageTextureCache, pixelBuffer, nil, pixelFormat, width, height, planeIndex, &texture)
        
        if status != kCVReturnSuccess {
            texture = nil
        }
        
        return texture
    }
    
    // Update image plane for camera only when viewport size changes
    func updateImagePlane(frame: ARFrame) {
        // Update the texture coordinates of our image plane to aspect fill the viewport
        let displayToCameraTransform = frame.displayTransform(for: .landscapeRight, viewportSize: viewportSize).inverted()
        
        let vertexData = imagePlaneVertexBuffer.contents().assumingMemoryBound(to: Float.self)
        for index in 0...3 {
            let textureCoordIndex = 4 * index + 2
            let textureCoord = CGPoint(x: CGFloat(kImagePlaneVertexData[textureCoordIndex]), y: CGFloat(kImagePlaneVertexData[textureCoordIndex + 1]))
            let transformedCoord = textureCoord.applying(displayToCameraTransform)
            vertexData[textureCoordIndex] = Float(transformedCoord.x)
            vertexData[textureCoordIndex + 1] = Float(transformedCoord.y)
        }
    }
    
    // Camera frame draw call
    func drawCapturedImage(renderEncoder: MTLRenderCommandEncoder) {
        guard let textureY = capturedImageTextureY, let textureCbCr = capturedImageTextureCbCr else {
            return
        }
        
        // Push a debug group allowing us to identify render commands in the GPU Frame Capture tool
        renderEncoder.pushDebugGroup("DrawCapturedImage")
        
        // Set render command encoder state
        renderEncoder.setCullMode(.none)
        renderEncoder.setRenderPipelineState(capturedImagePipelineState)
        renderEncoder.setDepthStencilState(capturedImageDepthState)
        
        // Set mesh's vertex buffers
        renderEncoder.setVertexBuffer(imagePlaneVertexBuffer, offset: 0, index: Int(kBufferIndexMeshPositions.rawValue))
        
        // Set any textures read/sampled from our render pipeline
        renderEncoder.setFragmentTexture(CVMetalTextureGetTexture(textureY), index: Int(kTextureIndexY.rawValue))
        renderEncoder.setFragmentTexture(CVMetalTextureGetTexture(textureCbCr), index: Int(kTextureIndexCbCr.rawValue))
        
        // Draw each submesh of our mesh
        renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
        
        renderEncoder.popDebugGroup()
    }
    
    // Point cloud draw call
    func drawPointCloudGeometry(renderEncoder: MTLRenderCommandEncoder) {
        guard anchorInstanceCount > 0 else {
            return
        }
        
        
        
        // Push a debug group allowing us to identify render commands in the GPU Frame Capture tool
        renderEncoder.pushDebugGroup("DrawPointClouds")
        
        // Set render command encoder state
        renderEncoder.setCullMode(.back)
        renderEncoder.setRenderPipelineState(pointCloudPipelineState)
        renderEncoder.setDepthStencilState(pointCloudDepthState)
        
        // Set any buffers fed into our render pipeline
        renderEncoder.setVertexBuffer(anchorUniformBuffer, offset: anchorUniformBufferOffset, index: Int(kBufferIndexInstanceUniforms.rawValue))
        renderEncoder.setVertexBuffer(sharedUniformBuffer, offset: sharedUniformBufferOffset, index: Int(kBufferIndexSharedUniforms.rawValue))
        renderEncoder.setFragmentBuffer(sharedUniformBuffer, offset: sharedUniformBufferOffset, index: Int(kBufferIndexSharedUniforms.rawValue))
        
        
        if(Int(currentRenderFrame.pointee.num_points) > 0)
        {
            
            
            renderFrameCenterBuffer =  device.makeBuffer(bytes: currentRenderFrame.pointee.center_list, length: Int(currentRenderFrame.pointee.num_points) * MemoryLayout<vector_float4>.size, options: [.storageModeShared])
            renderFrameDepthBuffer = device.makeBuffer(bytes: currentRenderFrame.pointee.depth_bytes, length: Int(currentRenderFrame.pointee.num_points) * MemoryLayout<uint>.size, options: [.storageModeShared])
            renderFrameColorBuffer = device.makeBuffer(bytes: currentRenderFrame.pointee.color_bytes, length: Int(currentRenderFrame.pointee.num_points) * MemoryLayout<uint>.size, options: [.storageModeShared])
            renderEncoder.setVertexBuffer(renderFrameCenterBuffer, offset: 0, index:0)
            renderEncoder.setVertexBuffer(renderFrameDepthBuffer, offset: 0, index:1)
            renderEncoder.setVertexBuffer(renderFrameColorBuffer, offset: 0, index:2)
            
            renderEncoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: Int(currentRenderFrame.pointee.num_points) , instanceCount: anchorInstanceCount)
            
            
        }
        
        
        
        
        renderEncoder.popDebugGroup()
    }
}

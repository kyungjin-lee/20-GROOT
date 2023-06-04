//
//  ViewController.swift
//  GrootPlayer
//
//  Created by Kyungjin Lee on 2020/06/10.
//  Copyright © 2020 Kyungjin Lee. All rights reserved.
//

import UIKit
import Metal
import MetalKit
import ARKit

extension MTKView : RenderDestinationProvider {
}

class ViewController: UIViewController, MTKViewDelegate, ARSessionDelegate {
    
    var session: ARSession!
    var renderer: Renderer!
    var userNameValue: String = ""
    var isSaveViewTrace: Bool = false
    var datasetName: String = ""
    var initialDepth: String = ""

    
    // dataset configure
    var filePrefix: String = ""
    var filePostfix: String = ""
    var fileStartIdx = 0
    var fileEndIdx = 0
    var fileList: [String] = []
    
    var isAnchored: Bool = false
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        // Set the view's delegate
        session = ARSession()
        session.delegate = self
        
       // self.writeModelFileURL = FileManager.documentsDirectoryURL.appendingPathComponent(userNameValue + "_model").appendingPathExtension("txt")
               
              
        
        // Set the view to use the default device
        if let view = self.view as? MTKView {
            view.device = MTLCreateSystemDefaultDevice()
            view.backgroundColor = UIColor.clear
            view.delegate = self
            
            guard view.device != nil else {
                print("Metal is not supported on this device")
                return
            }
            
            // prepare files to render
            configureSession(dataset: datasetName)
            var depthValue = (initialDepth as NSString).floatValue
            // Configure the renderer to draw to the view
            renderer = Renderer(session: session, metalDevice: view.device!, renderDestination: view, userName: userNameValue, isSaveViewTrace: isSaveViewTrace, fileList: fileList, initialDepth: depthValue)
            
            renderer.drawRectResized(size: view.bounds.size)
            

        }
        
        print("Username ", userNameValue)
        let tapGesture = UITapGestureRecognizer(target: self, action: #selector(ViewController.handleTap(gestureRecognize:)))
        view.addGestureRecognizer(tapGesture)
    }
    
    /*
     Configure session by setting filename for different datasets
     Need to be modified for custom datasets.
     */
    func configureSession(dataset: String)
    {
        var loop = 50
        if(dataset == "longdress")
        {
            filePrefix = "longdress/longdress_vox10_"
            //filePostfix = "_scaled_enc"
            filePostfix = "_scaled_enc"
            fileStartIdx = 1051
            fileEndIdx = 1350
            
        }
        else if(dataset == "twopeople")
        {
            filePrefix = "twopeople/twopeople_"
            filePostfix = "_enc"
            fileStartIdx = 0
            fileEndIdx = 299
        }
        else if(dataset == "band")
        {
            filePrefix = "band/ptcloud_hd0000"
            filePostfix = "_denoised_quantized_enc"
            fileStartIdx = 3000
            fileEndIdx = 6000
            
        }
        else if(dataset == "pizza")
        {
            filePrefix = "pizza/ptcloud_hd0000"
            filePostfix = "_denoised_quantized_enc"
            fileStartIdx = 3000
            fileEndIdx = 4299
        }
        else{
            print("Invalid dataset")
        }
        
        for j in 0..<loop
        {
        for i in fileStartIdx...fileEndIdx
        {
            fileList.append(filePrefix + String(format: "%03d", i) + filePostfix)
            //fileList.append(filePrefix + String(i) + filePostfix)
        }
        }
    }
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a session configuration
        let configuration = ARWorldTrackingConfiguration()

        // Run the view's session
        session.run(configuration)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        // Pause the view's session
        session.pause()
    }
    
    /*
     Important: Render starts when user first taps the screen
     */
    @objc
    func handleTap(gestureRecognize: UITapGestureRecognizer) {
        if(isAnchored)
        {
            return
        }
        
        if let currentFrame = session.currentFrame {
                   
                   //translation.columns.3.z = -0.2
                   var transform = matrix_identity_float4x4
                   // Add a new anchor to the session
                   let anchor = ARAnchor(transform: transform)
                   session.add(anchor: anchor)
                   isAnchored = true
                   
               }
     
    }
    
    // MARK: - MTKViewDelegate
    
    // Called whenever view changes orientation or layout is changed
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        renderer.drawRectResized(size: size)
    }
    
    // Called whenever the view needs to render
    func draw(in view: MTKView) {
        renderer.update()
    }
    
    // MARK: - ARSessionDelegate
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user
        
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        // Inform the user that the session has been interrupted, for example, by presenting an overlay
        
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        // Reset tracking and/or remove existing anchors if consistent tracking is required
        
    }
}

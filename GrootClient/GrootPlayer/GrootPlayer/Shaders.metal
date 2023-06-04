//
//  Shaders.metal
//  GrootPlayer
//
//  Created by Kyungjin Lee on 2020/06/10.
//  Copyright © 2020 Kyungjin Lee. All rights reserved.
//



#include <metal_stdlib>
#include <simd/simd.h>

// Include header shared between this Metal shader code and C code executing Metal API commands
#import "ShaderTypes.h"

using namespace metal;

typedef struct {
    float2 position [[attribute(kVertexAttributePosition)]];
    float2 texCoord [[attribute(kVertexAttributeTexcoord)]];
} ImageVertex;


typedef struct {
    float4 position [[position]];
    float2 texCoord;
} ImageColorInOut;


// Captured image vertex function
vertex ImageColorInOut capturedImageVertexTransform(ImageVertex in [[stage_in]]) {
    ImageColorInOut out;
    
    // Pass through the image vertex's position
    out.position = float4(in.position, 0.0, 1.0);
    
    // Pass through the texture coordinate
    out.texCoord = in.texCoord;
    
    return out;
}

// Captured image fragment function
fragment float4 capturedImageFragmentShader(ImageColorInOut in [[stage_in]],
                                            texture2d<float, access::sample> capturedImageTextureY [[ texture(kTextureIndexY) ]],
                                            texture2d<float, access::sample> capturedImageTextureCbCr [[ texture(kTextureIndexCbCr) ]]) {
    
    constexpr sampler colorSampler(mip_filter::linear,
                                   mag_filter::linear,
                                   min_filter::linear);
    
    const float4x4 ycbcrToRGBTransform = float4x4(
                                                  float4(+1.0000f, +1.0000f, +1.0000f, +0.0000f),
                                                  float4(+0.0000f, -0.3441f, +1.7720f, +0.0000f),
                                                  float4(+1.4020f, -0.7141f, +0.0000f, +0.0000f),
                                                  float4(-0.7010f, +0.5291f, -0.8860f, +1.0000f)
                                                  );
    
    // Sample Y and CbCr textures to get the YCbCr color at the given texture coordinate
    float4 ycbcr = float4(capturedImageTextureY.sample(colorSampler, in.texCoord).r,
                          capturedImageTextureCbCr.sample(colorSampler, in.texCoord).rg, 1.0);
    
    // Return converted RGB color
    return ycbcrToRGBTransform * ycbcr;
    // Use this to temporarily turn of background
    //float4 white = float4(1.0,1.0,1.0,1.0);
    //return white;
}


typedef struct {
    float3 position [[attribute(kVertexAttributePosition)]];
    float2 texCoord [[attribute(kVertexAttributeTexcoord)]];
    half3 normal    [[attribute(kVertexAttributeNormal)]];
} Vertex;


typedef struct {
    float4 position [[position]];
    float4 color;
    half3  eyePosition;
    half3  normal;
} ColorInOut;


// Anchor geometry vertex function
vertex ColorInOut anchorGeometryVertexTransform(Vertex in [[stage_in]],
                                                constant SharedUniforms &sharedUniforms [[ buffer(kBufferIndexSharedUniforms) ]],
                                                constant InstanceUniforms *instanceUniforms [[ buffer(kBufferIndexInstanceUniforms) ]],
                                                ushort vid [[vertex_id]],
                                                ushort iid [[instance_id]]) {
    ColorInOut out;
    
    // Make position a float4 to perform 4x4 matrix math on it
    float4 position = float4(in.position, 1.0);
    
    float4x4 modelMatrix = instanceUniforms[iid].modelMatrix;
    float4x4 modelViewMatrix = sharedUniforms.viewMatrix * modelMatrix;
    
    // Calculate the position of our vertex in clip space and output for clipping and rasterization
    out.position = sharedUniforms.projectionMatrix * modelViewMatrix * position;
    
    // Color each face a different color
    ushort colorID = vid / 4 % 6;
    out.color = colorID == 0 ? float4(0.0, 1.0, 0.0, 1.0) // Right face
    : colorID == 1 ? float4(1.0, 0.0, 0.0, 1.0) // Left face
    : colorID == 2 ? float4(0.0, 0.0, 1.0, 1.0) // Top face
    : colorID == 3 ? float4(1.0, 0.5, 0.0, 1.0) // Bottom face
    : colorID == 4 ? float4(1.0, 1.0, 0.0, 1.0) // Back face
    : float4(1.0, 1.0, 1.0, 1.0); // Front face
    
    // Calculate the positon of our vertex in eye space
    out.eyePosition = half3((modelViewMatrix * position).xyz);
    
    // Rotate our normals to world coordinates
    float4 normal = modelMatrix * float4(in.normal.x, in.normal.y, in.normal.z, 0.0f);
    out.normal = normalize(half3(normal.xyz));
    
    return out;
}

// Anchor geometry fragment function
fragment float4 anchorGeometryFragmentLighting(ColorInOut in [[stage_in]],
                                               constant SharedUniforms &uniforms [[ buffer(kBufferIndexSharedUniforms) ]]) {
    
    float3 normal = float3(in.normal);
    
    // Calculate the contribution of the directional light as a sum of diffuse and specular terms
    float3 directionalContribution = float3(0);
    {
        // Light falls off based on how closely aligned the surface normal is to the light direction
        float nDotL = saturate(dot(normal, -uniforms.directionalLightDirection));
        
        // The diffuse term is then the product of the light color, the surface material
        // reflectance, and the falloff
        float3 diffuseTerm = uniforms.directionalLightColor * nDotL;
        
        // Apply specular lighting...
        
        // 1) Calculate the halfway vector between the light direction and the direction they eye is looking
        float3 halfwayVector = normalize(-uniforms.directionalLightDirection - float3(in.eyePosition));
        
        // 2) Calculate the reflection angle between our reflection vector and the eye's direction
        float reflectionAngle = saturate(dot(normal, halfwayVector));
        
        // 3) Calculate the specular intensity by multiplying our reflection angle with our object's
        //    shininess
        float specularIntensity = saturate(powr(reflectionAngle, uniforms.materialShininess));
        
        // 4) Obtain the specular term by multiplying the intensity by our light's color
        float3 specularTerm = uniforms.directionalLightColor * specularIntensity;
        
        // Calculate total contribution from this light is the sum of the diffuse and specular values
        directionalContribution = diffuseTerm + specularTerm;
    }
    
    // The ambient contribution, which is an approximation for global, indirect lighting, is
    // the product of the ambient light intensity multiplied by the material's reflectance
    float3 ambientContribution = uniforms.ambientLightColor;
    
    // Now that we have the contributions our light sources in the scene, we sum them together
    // to get the fragment's lighting value
    float3 lightContributions = ambientContribution + directionalContribution;
    
    // We compute the final color by multiplying the sample from our color maps by the fragment's
    // lighting value
    float3 color = in.color.rgb * lightContributions;
    
    // We use the color we just computed and the alpha channel of our
    // colorMap for this fragment's alpha value
    return float4(color, in.color.w);
}

struct PointCloudVertexIn {
    float4 center[[attribute(0)]];
    uchar4 depth_bytes[[attribute(1)]];
    uchar4 color_bytes[[attribute(2)]];
};

struct PointCloudVertexOut {
    float4 position[[position]];
    float4 color;
    half3 eyePosition;
    float pointSize[[point_size]];
};

float4 get_child_center(float4 center, float sidelength, uchar byte)
{
    float4 childcenter = center;
    float sidelength_ = sidelength / 4;
    if(byte == 1)
    {
        childcenter[0] -= sidelength_;
        childcenter[1] -= sidelength_;
        childcenter[2] -= sidelength_;
    }
    else if(byte == 2)
    {
        childcenter[0] -= sidelength_;
        childcenter[1] -= sidelength_;
        childcenter[2] += sidelength_;
        
    }
    else if(byte == 3)
    {
        childcenter[0] -= sidelength_;
        childcenter[1] += sidelength_;
        childcenter[2] -= sidelength_;
        
    }
    else if(byte == 4)
    {
        childcenter[0] -= sidelength_;
        childcenter[1] += sidelength_;
        childcenter[2] += sidelength_;
        
    }
    else if(byte == 5)
    {
        childcenter[0] += sidelength_;
        childcenter[1] -= sidelength_;
        childcenter[2] -= sidelength_;
        
    }
    else if(byte == 6)
    {
        childcenter[0] += sidelength_;
        childcenter[1] -= sidelength_;
        childcenter[2] += sidelength_;
        
    }
    else if(byte == 7)
    {
        childcenter[0] += sidelength_;
        childcenter[1] += sidelength_;
        childcenter[2] -= sidelength_;
        
    }
    else if(byte == 8)
    {
        childcenter[0] += sidelength_;
        childcenter[1] += sidelength_;
        childcenter[2] += sidelength_;
        
    }
    else
    {
        childcenter[0] = 0;
        childcenter[1] = 0;
        childcenter[2] = 0;
    }
    
    return childcenter;
    
}
float4 calculate_points(float4 center, uchar3 bytes, float sidelength)
{
    float4 current_center = center;
    float current_sidelength = sidelength;
    float4 next_center = get_child_center(current_center, current_sidelength, bytes[0]);
    current_sidelength = current_sidelength / 2;
    current_center = next_center;
    next_center = get_child_center(current_center, current_sidelength, bytes[1]);
    current_sidelength = current_sidelength / 2;
    current_center = next_center;
    next_center = get_child_center(current_center, current_sidelength, bytes[2]);
    
    return next_center;
}
vertex PointCloudVertexOut pointcloud_vertex_shader(PointCloudVertexIn vertices [[stage_in]], constant SharedUniforms &sharedUniforms [[ buffer(kBufferIndexSharedUniforms) ]],
                                                    constant InstanceUniforms *instanceUniforms [[ buffer(kBufferIndexInstanceUniforms) ]],
                                                    unsigned int vid [[vertex_id]],
                                                    unsigned int iid [[instance_id]])
{
    
    
    
    
    PointCloudVertexOut out;
    
    // decode 4 bit depth bytes back to uint8_t
    uchar3 decoded_depth_bytes(vertices.depth_bytes[0],vertices.depth_bytes[1],vertices.depth_bytes[2]);
    if(vid % 2 == 1)
    {
        decoded_depth_bytes[0] = vertices.depth_bytes[0] >> 4;
        decoded_depth_bytes[1] = (vertices.depth_bytes[1] ) & 15;
        decoded_depth_bytes[2] = vertices.depth_bytes[1] >> 4;
        
    }
    else
    {
        decoded_depth_bytes[0] = (vertices.depth_bytes[0]) & 15;
        decoded_depth_bytes[1] = vertices.depth_bytes[0] >> 4 ;
        decoded_depth_bytes[2] = (vertices.depth_bytes[1] ) & 15;
        
    }
    
    // use depth bytes to calculate final x, y, z coordinates
    float4 position = calculate_points(vertices.center,decoded_depth_bytes, sharedUniforms.sidelength);
    
    
    // position[1] = position[1] * (-1);
    float4x4 modelMatrix = instanceUniforms[iid].modelMatrix;
    
    float4x4 modelViewMatrix = sharedUniforms.viewMatrix * modelMatrix;
    out.position = sharedUniforms.projectionMatrix * modelViewMatrix * position;
    
    //float z_dist = out.position[2] / out.position[3];
    // out.pointSize = z_dist * 8;
    
    out.pointSize = 4;
    
    out.eyePosition = half3((modelViewMatrix * position).xyz);
    
    out.color = float4(float(vertices.color_bytes[0])/255.0, float(vertices.color_bytes[1])/255.0, float(vertices.color_bytes[2])/255.0, float(vertices.color_bytes[3]));
    return out;
}




fragment float4 pointcloud_fragment_shader(PointCloudVertexOut vertices [[stage_in]], float2 pointCoord [[point_coord]]){
    float4 outColor = vertices.color;
    return outColor;
}







//-------------------For point cloud geometry----------------------------
struct VertexOut {
    float4 position[[position]];
    float4 color;
    half3  eyePosition;
    float pointSize[[point_size]];
};


struct VertexIn{
    float3 position[[attribute(0)]];
    uint color[[attribute(1)]];
};





vertex VertexOut vertex_shader(VertexIn vertices [[stage_in]], constant SharedUniforms &sharedUniforms [[ buffer(kBufferIndexSharedUniforms) ]],
                               constant InstanceUniforms *instanceUniforms [[ buffer(kBufferIndexInstanceUniforms) ]],
                               unsigned int vid [[vertex_id]],
                               unsigned int iid [[instance_id]] ){
    VertexOut out;
    float3 temp = float3(vertices.position);
    float4 position = float4(temp, 1.0);
    
    
    float4x4 modelMatrix = instanceUniforms[iid].modelMatrix;
    float4x4 modelViewMatrix = sharedUniforms.viewMatrix * modelMatrix;
    out.position = sharedUniforms.projectionMatrix * modelViewMatrix * position;
    
    
    out.pointSize = 7;
    //   if(position[2] < -4) out.pointSize = 7;
    
    
    float4 color = unpack_unorm4x8_to_float(vertices.color);
    // float4 color = unpack_unorm4x8_srgb_to_float(vertices.color);
    // color[0] = color[0] / 255.0;
    //  color[1] = color[1] / 255.0;
    //  color[2] = color[2] / 255.0;
    /*if(temp[0] == 0 && temp[1] ==0 && temp[2] == 0)
     {
     color[4] = 0.0;
     }*/
    out.color = color;
    // out.color = vertices.color / 255.0;
    //out.color = float4(1.0,1.0,1.0,1.0);
    out.eyePosition = half3((modelViewMatrix * position).xyz);
    
    return out;
    
}




fragment float4 fragment_shader(VertexOut vertices [[stage_in]], float2 pointCoord [[point_coord]]){
    
    float4 outColor = vertices.color;
    
    //  outColor.a = 1.0 - smoothstep(0.4, 0.5, dist);
    //   return half4(outColor);
    // float4 outColor = vertices.color;
    return outColor;
}
//---------------------------------------------------------------




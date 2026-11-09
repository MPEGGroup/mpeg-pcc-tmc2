# PccGs4AppEncoder

## Overview

**PccGs4AppEncoder** is a Gaussian Splatting (GS) point cloud encoder application that is part of the MPEG Point Cloud Compression (PCC) Test Model 2 (TMC2) software. It encodes 3D Gaussian Splatting point clouds into V3C (Visual Volumetric Video-based Coding) bitstreams compliant with ISO/IEC 23090-5.

The encoder processes floating-point Gaussian Splatting point clouds (`PCCPointSetGS`) by first applying quantization and adaptation (`PCCAdaptation`) to convert them into integer-based point clouds (`PCCPointSet3`), which are then encoded using the main encoder pipeline (`PCCGSEncoder`).

## Features

- Encodes 3D Gaussian Splatting point clouds with associated attributes
- Supports geometry, opacity, spherical harmonics, scale, and rotation attributes
- Point cloud adaptation and quantization preprocessing (float → int conversion)
- Generates V3C compliant bitstreams
- Configurable video encoder settings for geometry and attributes
- Debug output support for adapted point clouds and configuration files

## Complete Encoding Pipeline

The encoding process consists of two main stages:

### Stage 1: Adaptation (Float → Integer)
The `PCCAdaptation` module converts floating-point Gaussian Splatting data into quantized integer representation:

1. **Input**: `PCCPointSetGS` (float precision)
   - Positions: 3D float coordinates
   - Opacity: float values
   - Spherical Harmonics: float coefficients (degree 0-3)
   - Scale: 3D float values
   - Rotation: quaternions (float)

2. **Adaptation Process**:
   - Quantize positions to integer coordinates based on `geometryBitDepth3D`
   - Quantize attributes to integer values based on attribute bit depths
   - Apply bounding box normalization
   - Store quantization parameters for decoder reconstruction

3. **Output**: `PCCPointSet3` (integer precision)
   - Quantized geometry and attributes ready for encoding

### Stage 2: Encoding (Integer → Bitstream)
The `PCCGSEncoder` encodes the adapted point cloud through the following steps:

1. **HLS Initialization**: Initialize V3C and Atlas high-level syntax structures
2. **Patch Generation**: Generate 3D patches and pack them into 2D layouts using CTC algorithm
3. **HLS Update**: Update VPS and ASPS with map dimensions from packing results
4. **Map Generation**: Create geometry and attribute 2D maps from patches
5. **Video Encoding**: Encode maps using HM/VTM video codecs
6. **Bitstream Writing**: Generate final V3C bitstream with all syntax elements

## Application Architecture

```mermaid
flowchart TB
    subgraph Application["PccGs4AppEncoder Application"]
        direction TB
        CLI["Command Line Parser"]
        Loader["Point Cloud Loader\n(PCCPointSetGS)"]
        Adaptation["PCCAdaptation\n(Float → Integer)"]
        Encoder["PCCGSEncoder\n(Integer → Bitstream)"]
        Writer["Bitstream Writer"]
    end

    Input["Input Point Cloud\n(.ply file)\nFloat Precision"]
    Config["Configuration Files\n(encoder + adaptation configs)"]
    Output["V3C Bitstream\n(.bin file)"]
    Debug["Debug Outputs\n(adapted .ply + config.json)"]

    Input --> Loader
    Config --> CLI
    CLI --> Loader
    Loader --> |PCCPointSetGS| Adaptation
    Adaptation --> |PCCPointSet3| Encoder
    Adaptation -.-> Debug
    Encoder --> Writer
    Writer --> Output

    style Application fill:#e1f5fe
    style Encoder fill:#fff3e0,stroke:#ff9800,stroke-width:2px
    style Adaptation fill:#f3e5f5,stroke:#9c27b0,stroke-width:2px
```

## PCCGSEncoder Component Architecture

The encoder follows the **Facade design pattern**, orchestrating four specialized component classes:

```mermaid
flowchart TB
    subgraph PCCGSEncoder["PCCGSEncoder (Facade)"]
        direction TB
        
        subgraph Components["Component Classes"]
            HLS["PCCGSHlsBuilder\n─────────────\n• Initialize V3C HLS\n• Initialize Atlas HLS\n• Update VPS\n• Update ASPS\n• Add Atlas Frames"]
            Patch["PCCGSPatchGenerator\n─────────────\n• Generate 3D Patches\n• Pack to 2D (CTC)\n• Morton Ordering\n• Compute 2D Positions"]
            Map["PCCGSMapGenerator\n─────────────\n• Generate Geometry Maps\n• Generate Attribute Maps\n• Generate Packed Maps"]
            Video["PCCGSVideoEncoder\n─────────────\n• Encode Geometry Video\n• Encode Attribute Videos\n• Encode Packed Videos"]
        end
    end

    PointCloud["Input Point Cloud\n(PCCPointSet3)"]
    Params["Encoder Parameters\n(PCCGSEncoderParameters)"]
    SSVU["Output Bitstream\n(SampleStreamV3CUnit)"]

    PointCloud --> PCCGSEncoder
    Params --> PCCGSEncoder
    PCCGSEncoder --> SSVU

    HLS --> Patch
    Patch --> Map
    Map --> Video

    style PCCGSEncoder fill:#fff3e0,stroke:#ff9800,stroke-width:2px
    style Components fill:#e8f5e9
```

## Encoding Pipeline Flow

```mermaid
sequenceDiagram
    participant App as PccGs4AppEncoder
    participant Adapt as PCCAdaptation
    participant Enc as PCCGSEncoder
    participant HLS as PCCGSHlsBuilder
    participant Patch as PCCGSPatchGenerator
    participant Map as PCCGSMapGenerator
    participant Vid as PCCGSVideoEncoder
    participant Writer as PCCBitstreamWriter

    loop For Each Frame
        App->>App: Load Point Cloud (.ply)<br/>PCCPointSetGS (float)
        
        rect rgb(243, 229, 245)
            Note over App,Adapt: Adaptation Stage
            App->>Adapt: adapt(originalGS, adaptedPC, adaptationParams)
            Adapt->>Adapt: Quantize Geometry<br/>(float → int)
            Adapt->>Adapt: Quantize Attributes<br/>(float → int)
            Adapt-->>App: PCCPointSet3 (integer)
            App->>App: Save adapted.ply (debug)
            App->>App: Save config.json (debug)
        end
        
        rect rgb(255, 243, 224)
            Note over Enc,Writer: PCCGSEncoder Pipeline
            App->>Enc: encode(pointCloud, params, adaptationParams, ssvu, logger)
            
            Enc->>HLS: initialize(hls, bitstreamStat, params, adaptationParams)
            HLS-->>Enc: V3C/Atlas HLS initialized
            
            Enc->>Patch: generate(pointCloud, params)
            Patch->>Patch: Generate 3D Patches
            Patch->>Patch: Pack to 2D (CTC)
            Patch->>Patch: Morton Sort Points
            Patch-->>Enc: PatchResult (patches, mapWidth, mapHeight)
            
            Enc->>HLS: addAtlasFrame(hls, patches, params)
            Enc->>HLS: updateVps(hls, mapWidth, mapHeight, params)
            Enc->>HLS: updateAsps(hls, mapWidth, mapHeight, params)
            
            Enc->>Map: generate(pointCloud, patches, hls, mapWidth, mapHeight)
            Map->>Map: Generate Geometry Maps
            Map->>Map: Generate Attribute Maps (SH, opacity, scale, rotation)
            Map->>Map: Generate Packed Maps (optional)
            Map-->>Enc: VideoFrames (geometry, attributes, packed)
            
            Enc->>Vid: encode(videoFrames, hls, params)
            Vid->>Vid: Encode Geometry Video (HM/VTM)
            Vid->>Vid: Encode Attribute Videos (HM/VTM)
            Vid->>Vid: Encode Packed Videos (optional)
            Vid-->>Enc: Encoded video bitstreams
            
            Enc->>Writer: encode(hls, ssvu)
            Writer->>Writer: Generate V3C Units
            Writer->>Writer: Write VPS/ASPS/AFPS
            Writer->>Writer: Write Atlas Tile Data
            Writer-->>Enc: V3C bitstream generated
        end
        
        Enc-->>App: Encoding complete
        App->>App: Save bitstream (.bin)
    end
```

## Data Flow Diagram

```mermaid
flowchart LR
    subgraph Input
        PCGS["Point Cloud GS\n(PCCPointSetGS)\n─────────\nPositions (float)\nOpacity (float)\nSH Coefficients (float)\nScale (float)\nRotation (quaternion)"]
    end
    
    subgraph Adaptation["Adaptation Stage"]
        Quant["Quantization\n─────────\ngeometryBitDepth3D\nattributeBitDepths"]
        PC3["Point Cloud\n(PCCPointSet3)\n─────────\nPositions (int)\nOpacity (int)\nSH Coefficients (int)\nScale (int)\nRotation (int)"]
    end

    subgraph Patches["Patch Generation"]
        P3D["3D Patches\n(Bounding Boxes)"]
        P2D["2D Packing\n(CTC Algorithm)"]
        Sort["Morton Sort"]
    end

    subgraph Maps["Map Generation"]
        GeoMap["Geometry Map\n(X, Y, Z)"]
        AttrMap["Attribute Maps\n─────────\nOpacity\nSH (16 partitions)\nScale\nRotation"]
        PackMap["Packed Maps\n(Optional)"]
    end

    subgraph Encoding["Video Encoding"]
        VidEnc["HM/VTM Encoder"]
    end

    subgraph Output
        BS["V3C Bitstream\n─────────\nVPS\nAtlas Data\nGeometry Video\nAttribute Videos"]
    end

    PCGS --> Quant --> PC3
    PC3 --> P3D --> P2D --> Sort
    Sort --> GeoMap
    Sort --> AttrMap
    GeoMap --> PackMap
    AttrMap --> PackMap
    GeoMap --> VidEnc
    AttrMap --> VidEnc
    PackMap --> VidEnc
    VidEnc --> BS

    style PCGS fill:#e3f2fd
    style Adaptation fill:#f3e5f5
    style BS fill:#e8f5e9
```

## Class Diagram

```mermaid
classDiagram
    class PCCGSEncoder {
        -PCCGSHlsBuilder hlsBuilder_
        -PCCGSPatchGenerator patchGenerator_
        -PCCGSMapGenerator mapGenerator_
        -PCCGSVideoEncoder videoEncoder_
        -PCCHighLevelSyntax hls_
        -PCCBitstreamStat bitstreamStat_
        +encode(pointCloud, params, adaptationParams, ssvu, logger) int
    }

    class PCCGSHlsBuilder {
        +initialize(hls, bitstreamStat, params, adaptationParams)
        +addAtlasFrame(hls, patches, params)
        +updateVps(hls, mapWidth, mapHeight, params)
        +updateAsps(hls, mapWidth, mapHeight, params)
        -initializeV3CHls(hls, params)
        -initializeAtlasHls(hls, params)
    }

    class PCCGSPatchGenerator {
        -patches_ : vector~GSEncoderPatchInformation~
        +generate(source, params) PCCGSPatchResult
        -generate3DPatches(source, params)
        -generate2DPatchesCTC(source, params)
        -packRawPatches2DCTC()
        -sortPointsInPatchMorton(source)
    }

    class PCCGSMapGenerator {
        +generate(source, patches, hls, width, height) PCCGSVideoFrames
        -generateGeometryMap(source, patches, image)
        -generateAttributeMap(patches, image, componentIdx, attribute)
        -generatePackedMaps(hls, packedVideoFrames)
    }

    class PCCGSVideoEncoder {
        +encode(frames, hls, params)
        -encodeVideo(videoFrames, videoType, params)
        -getVideoTypeByAttributeType(attrType, partitionIdx) PCCVideoType
    }

    class PCCGSEncoderParameters {
        +geometryQP_ : size_t
        +attributeQP_ : size_t
        +packQP_ : size_t
        +geometryBitDepth3D_ : size_t
        +geometryBitDepth2D_ : size_t
        +packGeometry_ : bool
        +packAttributes_ : bool
    }

    class GSEncoderPatchInformation {
        +patch : PCCRawPointsPatch
        +pointsIndicesInPatch : vector~size_t~
        +pointsPositionsIn2D : array~vector~Pixel2D~, 3~
    }

    PCCGSEncoder *-- PCCGSHlsBuilder
    PCCGSEncoder *-- PCCGSPatchGenerator
    PCCGSEncoder *-- PCCGSMapGenerator
    PCCGSEncoder *-- PCCGSVideoEncoder
    PCCGSEncoder ..> PCCGSEncoderParameters
    PCCGSPatchGenerator ..> GSEncoderPatchInformation
```

## Usage

```bash
./bin/PccGs4AppEncoder \
	--uncompressedDataPath=/path/to/ply_file.ply \
   	--configurationFolder=./mpeg-pcc-tmc2/cfg/gs4/ \
	--config=./mpeg-pcc-tmc2/cfg/gs4/hm-main10/gs-main10-common.cfg \
	--config=./mpeg-pcc-tmc2/cfg/gs4/gs-pack-none.cfg \
	--config=./mpeg-pcc-tmc2/cfg/gs4/hm-main10/rate/r04.cfg \
	--compressedStreamPath=/path/to/compressed_file.v3c \
```

### Key Parameters

| Parameter | Description |
|-----------|-------------|
| `--uncompressedDataPath` | Path template for input point cloud files |
| `--compressedStreamPath` | Output V3C bitstream file path |
| `--configurationFolder` | Folder containing encoder configuration files |
| `--geometryConfig` | Geometry video encoder configuration file |
| `--attributeConfig` | Attribute video encoder configuration file |
| `--packConfig` | Packed video encoder configuration file |
| `--geometryQP` | Quantization parameter for geometry (default: 6) |
| `--attributeQP` | Quantization parameter for attributes (default: 6) |
| `--packQP_ ` | Quantization parameter for pack (default: 6) |

## Dependencies

- **PccLibGSEncoder**: Core GS encoding library
- **PccLibCommon**: Common utilities and data structures
- **PccLibBitstreamWriter**: V3C bitstream generation
- **PccLibAdaptation**: Point cloud preprocessing
- **HM/VTM**: HEVC/VVC video encoder for map encoding

## File Structure

```
source/app/PccGs4AppEncoder/
├── PccGs4AppEncoder.cpp      # Main application entry point
├── PCCGs4AppEncoderParams.h  # Application parameter handling
├── CMakeLists.txt           # Build configuration
└── README.md                # This file

source/lib/PccLibGSEncoder/
├── include/
│   ├── PccGSEncoder.h           # Main encoder facade
│   ├── PccGSEncoderParameters.h # Encoder parameters
│   ├── PCCGSPatchInfo.h         # Patch data structures
│   ├── PCCGSHlsBuilder.h        # HLS builder component
│   ├── PCCGSPatchGenerator.h    # Patch generator component
│   ├── PCCGSMapGenerator.h      # Map generator component
│   └── PCCGSVideoEncoder.h      # Video encoder component
└── source/
    ├── PccGSEncoder.cpp
    ├── PCCGSHlsBuilder.cpp
    ├── PCCGSPatchGenerator.cpp
    ├── PCCGSMapGenerator.cpp
    └── PCCGSVideoEncoder.cpp
```

## Build

```bash
# From the project root directory
./build.sh

# The executable will be in bin/PccGs4AppEncoder
```


#include "cppimg.h"
#include <algorithm>
#include <args.hxx>
#include <array>
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <openvdb/io/Stream.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <tuple>
#include <vector>

namespace convvdb
{
using OpenVDBHalf1Grid = openvdb::HalfGrid;
using OpenVDBFloat1Grid = openvdb::FloatGrid;
using OpenVDBFloat2Grid = openvdb::Grid<openvdb::tree::Tree4<openvdb::Vec2f, 5, 4, 3>::Type>;
using OpenVDBFloat3Grid = openvdb::Vec3SGrid;
using OpenVDBFloat4Grid = openvdb::Grid<openvdb::tree::Tree4<openvdb::Vec4f, 5, 4, 3>::Type>;
using OpenVDBDouble1Grid = openvdb::DoubleGrid;
using OpenVDBDouble2Grid = openvdb::Grid<openvdb::tree::Tree4<openvdb::Vec2d, 5, 4, 3>::Type>;
using OpenVDBDouble3Grid = openvdb::Vec3DGrid;
using OpenVDBDouble4Grid = openvdb::Grid<openvdb::tree::Tree4<openvdb::Vec4d, 5, 4, 3>::Type>;

struct IntVector3
{
    int32_t x_;
    int32_t y_;
    int32_t z_;
};

IntVector3 operator-(const IntVector3& x0, const IntVector3& x1)
{
    return {
        x0.x_ - x1.x_, x0.y_ - x1.y_, x0.z_ - x1.z_};
}

struct Vector4
{
    Vector4 operator-() const
    {
        return {-x_, -y_, -z_, -w_};
    }

    bool isNearlyZero(float tolerance = 1.0e-8f) const
    {
        return ::abs(x_) <= tolerance && ::abs(y_) <= tolerance && ::abs(z_) <= tolerance;
    }
    void normalize(float tolerance = 1.0e-8f)
    {
        float l = ::sqrtf(x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_);
        if(l <= tolerance) {
            x_ = y_ = z_ = 0.0f;
            w_ = 1.0f;
            return;
        }
        l = 1.0f / l;
        x_ *= l;
        y_ *= l;
        z_ *= l;
        w_ *= l;
    }

    float x_;
    float y_;
    float z_;
    float w_;
};

struct Transform
{
    Vector4 rotation_;
    Vector4 translation_;
    Vector4 scale_;
};

enum class EOpenVDBGridType : uint8_t
{
    Unknown = 0,
    Half,
    Float,
    Double,
};

struct OpenVDBGridInfo
{
    Transform transform;
    IntVector3 volumeActiveAABBMin;
    IntVector3 volumeActiveAABBMax;
    IntVector3 volumeActiveDim;
    std::string name;
    std::string displayString; // Contains Index (into source file grids), type and Name
    uint32_t index;
    uint32_t numComponents;
    EOpenVDBGridType type;
    bool isInWorldSpace;
};

Vector4 extractScaling(openvdb::Mat4d& unscaled, const openvdb::Mat4d& mat, float tolerance = 1.0e-8f)
{
    const float squareSum0 = float((mat[0][0] * mat[0][0]) + (mat[0][1] * mat[0][1]) + (mat[0][2] * mat[0][2]));
    const float squareSum1 = float((mat[1][0] * mat[1][0]) + (mat[1][1] * mat[1][1]) + (mat[1][2] * mat[1][2]));
    const float squareSum2 = float((mat[2][0] * mat[2][0]) + (mat[2][1] * mat[2][1]) + (mat[2][2] * mat[2][2]));

    unscaled = mat;
    Vector4 scale = {};
    if(squareSum0 > tolerance) {
        float scale0 = ::sqrtf(squareSum0);
        scale.x_ = scale0;
        float InvScale0 = 1.f / scale0;
        unscaled[0][0] *= InvScale0;
        unscaled[0][1] *= InvScale0;
        unscaled[0][2] *= InvScale0;
    }

    if(squareSum1 > tolerance) {
        float scale1 = ::sqrtf(squareSum1);
        scale.y_ = scale1;
        float InvScale1 = 1.f / scale1;
        unscaled[1][0] *= InvScale1;
        unscaled[1][1] *= InvScale1;
        unscaled[1][2] *= InvScale1;
    }

    if(squareSum2 > tolerance) {
        float scale2 = ::sqrtf(squareSum2);
        scale.z_ = scale2;
        float InvScale2 = 1.f / scale2;
        unscaled[2][0] *= InvScale2;
        unscaled[2][1] *= InvScale2;
        unscaled[2][2] *= InvScale2;
    }

    return scale;
}

openvdb::Mat4d setAxis(openvdb::Mat4d m, int32_t i, const Vector4& axis)
{
    m[i][0] = axis.x_;
    m[i][1] = axis.y_;
    m[i][2] = axis.z_;
    return m;
}

enum class EAxis
{
    X,
    Y,
    Z,
    W
};

Vector4 getScaledAxis(const openvdb::Mat4d& m, EAxis axis)
{
    switch(axis) {
    case EAxis::X:
        return {(float)m[0][0], (float)m[0][1], (float)m[0][2], 0.0f};

    case EAxis::Y:
        return {(float)m[1][0], (float)m[1][1], (float)m[1][2], 0.0f};

    case EAxis::Z:
        return {(float)m[2][0], (float)m[2][1], (float)m[2][2], 0.0f};

    default:
        return {};
    }
}

Vector4 toQuaternion(const openvdb::Mat4d& mat)
{
    if(getScaledAxis(mat, EAxis::X).isNearlyZero() || getScaledAxis(mat, EAxis::Y).isNearlyZero() || getScaledAxis(mat, EAxis::Z).isNearlyZero()) {
        return {0.0f, 0.0f, 0.0f, 1.0f};
    }

    // const MeReal *const t = (MeReal *) tm;
    float s;

    // Check diagonal (trace)
    const float tr = float(mat[0][0] + mat[1][1] + mat[2][2]);

    Vector4 result;
    if(tr > 0.0f) {
        float invS = 1.0f / ::sqrtf(tr + 1.f);
        result.w_ = 0.5f * (1.f / invS);
        s = 0.5f * invS;

        result.x_ = float((mat[1][2] - mat[2][1]) * s);
        result.y_ = float((mat[2][0] - mat[0][2]) * s);
        result.z_ = float((mat[0][1] - mat[1][0]) * s);
    } else {
        // diagonal is negative
        int32_t i = 0;

        if(mat[1][1] > mat[0][0])
            i = 1;

        if(mat[2][2] > mat[i][i])
            i = 2;

        static constexpr int32_t nxt[3] = {1, 2, 0};
        const int32_t j = nxt[i];
        const int32_t k = nxt[j];

        s = float(mat[i][i] - mat[j][j] - mat[k][k]) + 1.0f;

        float invS = 1.0f / ::sqrtf(s);

        float qt[4];
        qt[i] = 0.5f * (1.f / invS);

        s = 0.5f * invS;

        qt[3] = float(mat[j][k] - mat[k][j]) * s;
        qt[j] = float(mat[i][j] + mat[j][i]) * s;
        qt[k] = float(mat[i][k] + mat[k][i]) * s;

        result.x_ = qt[0];
        result.y_ = qt[1];
        result.z_ = qt[2];
        result.w_ = qt[3];
    }
    return result;
}

Transform getTransform(const openvdb::Mat4d& mat)
{
    Transform transform;
    // Get the 3D scale from the matrix
    openvdb::Mat4d unscaled;
    transform.scale_ = extractScaling(unscaled, mat);

    // If there is negative scaling going on, we handle that here
    if(mat.det() < 0.0) {
        // Assume it is along X and modify transform accordingly.
        // It doesn't actually matter which axis we choose, the 'appearance' will be the same
        transform.scale_.x_ *= -1.0;
        Vector4 scaledAxis = -getScaledAxis(unscaled, EAxis::X);
        unscaled = setAxis(unscaled, 0, scaledAxis);
    }

    transform.rotation_ = toQuaternion(unscaled);
    transform.translation_ = {(float)mat[3][0], (float)mat[3][1], (float)mat[3][2], 0.0f};
    transform.rotation_.normalize();
    return transform;
}

static OpenVDBGridInfo getOpenVDBGridInfo(openvdb::GridBase::Ptr grid, uint32_t gridIndex)
{
    openvdb::CoordBBox volumeActiveAABB = grid->evalActiveVoxelBoundingBox();
    openvdb::Coord volumeActiveDim = grid->evalActiveVoxelDim();
    openvdb::math::MapBase::ConstPtr mapBase = grid->constTransform().baseMap();
    openvdb::Vec3d VoxelSize = mapBase->voxelSize();
    openvdb::Mat4d gridTransformVDB = mapBase->getAffineMap()->getConstMat4();
    OpenVDBGridInfo gridInfo;
    gridInfo.index = gridIndex;
    gridInfo.numComponents = 0;
    gridInfo.type = EOpenVDBGridType::Unknown;
    gridInfo.volumeActiveAABBMin = {volumeActiveAABB.min().x(), volumeActiveAABB.min().y(), volumeActiveAABB.min().z()};
    gridInfo.volumeActiveAABBMax = {volumeActiveAABB.max().x(), volumeActiveAABB.max().y(), volumeActiveAABB.max().z()};
    gridInfo.volumeActiveDim = {volumeActiveDim.x(), volumeActiveDim.y(), volumeActiveDim.z()};
    gridInfo.isInWorldSpace = grid->isInWorldSpace();

    gridInfo.transform = getTransform(gridTransformVDB);
    if(grid->isType<OpenVDBHalf1Grid>()) {
        gridInfo.numComponents = 1;
        gridInfo.type = EOpenVDBGridType::Half;
    } else if(grid->isType<OpenVDBFloat1Grid>()) {
        gridInfo.numComponents = 1;
        gridInfo.type = EOpenVDBGridType::Float;
    } else if(grid->isType<OpenVDBFloat2Grid>()) {
        gridInfo.numComponents = 2;
        gridInfo.type = EOpenVDBGridType::Float;
    } else if(grid->isType<OpenVDBFloat3Grid>()) {
        gridInfo.numComponents = 3;
        gridInfo.type = EOpenVDBGridType::Float;
    } else if(grid->isType<OpenVDBFloat4Grid>()) {
        gridInfo.numComponents = 4;
        gridInfo.type = EOpenVDBGridType::Float;
    } else if(grid->isType<OpenVDBDouble1Grid>()) {
        gridInfo.numComponents = 1;
        gridInfo.type = EOpenVDBGridType::Double;
    } else if(grid->isType<OpenVDBDouble2Grid>()) {
        gridInfo.numComponents = 2;
        gridInfo.type = EOpenVDBGridType::Double;
    } else if(grid->isType<OpenVDBDouble3Grid>()) {
        gridInfo.numComponents = 3;
        gridInfo.type = EOpenVDBGridType::Double;
    } else if(grid->isType<OpenVDBDouble4Grid>()) {
        gridInfo.numComponents = 4;
        gridInfo.type = EOpenVDBGridType::Double;
    }

    {
        gridInfo.name = grid->getName();
        gridInfo.displayString = std::format("{0}. type: {1}, name: \"{2}\"", gridInfo.index, (int32_t)gridInfo.type, gridInfo.name.c_str());
    }
    return gridInfo;
}

template<class T>
cppimg::DDS::Format toFormat()
{
    return cppimg::DDS::Format::UNKNOWN;
}

template<>
cppimg::DDS::Format toFormat<int8_t>()
{
    return cppimg::DDS::Format::R8_SNORM;
}

template<>
cppimg::DDS::Format toFormat<uint8_t>()
{
    return cppimg::DDS::Format::R8_UNORM;
}

template<>
cppimg::DDS::Format toFormat<openvdb::Half>()
{
    return cppimg::DDS::Format::R16_FLOAT;
}

template<>
cppimg::DDS::Format toFormat<openvdb::Vec2H>()
{
    return cppimg::DDS::Format::R16G16_FLOAT;
}

template<>
cppimg::DDS::Format toFormat<openvdb::Vec4H>()
{
    return cppimg::DDS::Format::R16G16B16A16_FLOAT;
}

template<>
cppimg::DDS::Format toFormat<float>()
{
    return cppimg::DDS::Format::R32_FLOAT;
}

template<>
cppimg::DDS::Format toFormat<openvdb::Vec2f>()
{
    return cppimg::DDS::Format::R32G32_FLOAT;
}

template<>
cppimg::DDS::Format toFormat<openvdb::Vec3f>()
{
    return cppimg::DDS::Format::R32G32B32_FLOAT;
}

template<>
cppimg::DDS::Format toFormat<openvdb::Vec4f>()
{
    return cppimg::DDS::Format::R32G32B32A32_FLOAT;
}

template<class T, class U>
void convert(const T& x0, U& x1)
{
    x1 = x0;
}

template<>
void convert(const double& x0, float& x1)
{
    x1 = static_cast<float>(x0);
}

template<>
void convert(const openvdb::Vec2d& x0, openvdb::Vec2f& x1)
{
    x1.x() = static_cast<float>(x0.x());
    x1.y() = static_cast<float>(x0.y());
}

template<>
void convert(const openvdb::Vec3d& x0, openvdb::Vec3f& x1)
{
    x1.x() = static_cast<float>(x0.x());
    x1.y() = static_cast<float>(x0.y());
    x1.z() = static_cast<float>(x0.z());
}

template<>
void convert(const openvdb::Vec4d& x0, openvdb::Vec4f& x1)
{
    x1.x() = static_cast<float>(x0.x());
    x1.y() = static_cast<float>(x0.y());
    x1.z() = static_cast<float>(x0.z());
    x1.w() = static_cast<float>(x0.w());
}

template<>
void convert(const openvdb::Half& x0, int8_t& x1)
{
    x1 = static_cast<float>(x0);
}

namespace
{
    int8_t toInt8(float x)
    {
        x = std::clamp(x, -1.0f, 1.0f);
        if(0.0f <= x) {
            int32_t xi = static_cast<int32_t>(x * 128.0f);
            return xi <= 127 ? static_cast<int8_t>(xi) : 127;
        } else {
            int32_t xi = static_cast<int32_t>(x * -129.0f);
            return -128 <= xi ? static_cast<int8_t>(xi) : -128;
        }
    }

    uint8_t toUint8(float x)
    {
        x = std::clamp(x, 0.0f, 1.0f);
        int32_t xi = static_cast<int32_t>(x * 256.0f);
        return xi <= 255 ? static_cast<uint8_t>(xi) : 255;
    }

    uint8_t toUint8(double x)
    {
        x = std::clamp(x, 0.0, 1.0);
        int32_t xi = static_cast<int32_t>(x * 256.0);
        return xi <= 255 ? static_cast<uint8_t>(xi) : 255;
    }
} // namespace

template<>
void convert(const openvdb::Half& x0, uint8_t& x1)
{
    assert(0.0f <= static_cast<float>(x0) && static_cast<float>(x0) <= 1.0f);
    x1 = toUint8(static_cast<float>(x0));
}

template<>
void convert(const float& x0, uint8_t& x1)
{
    assert(0.0f <= x0 && x0 <= 1.0f);
    x1 = toUint8(x0);
}

template<>
void convert(const double& x0, uint8_t& x1)
{
    assert(0.0 <= x0 && x0 <= 1.0);
    x1 = toUint8(x0);
}

template<class T>
void convertTo16(const T&, uint16_t&)
{
}

template<>
void convertTo16<openvdb::Half>(const openvdb::Half& x0, uint16_t& x1)
{
    x1 = x0.bits();
}

template<>
void convertTo16<float>(const float& x0, uint16_t& x1)
{
    openvdb::Half h(x0);
    x1 = h.bits();
}

template<>
void convertTo16<double>(const double& x0, uint16_t& x1)
{
    openvdb::Half h(static_cast<float>(x0));
    x1 = h.bits();
}

template<class T>
void convertTo32(const T&, float&)
{
}

template<>
void convertTo32<openvdb::Half>(const openvdb::Half& x0, float& x1)
{
    x1 = static_cast<float>(x0);
}

template<>
void convertTo32<float>(const float& x0, float& x1)
{
    x1 = x0;
}

template<>
void convertTo32<double>(const double& x0, float& x1)
{
    x1 = static_cast<float>(x0);
}

std::string getFilepath(const std::u8string& path, const std::u8string& basename, const std::u8string& output_dir, openvdb::GridBase::ConstPtr grid)
{
    std::string name = std::filesystem::path(path).stem().string();
    std::string_view digits = std::string_view(name).substr(basename.size());
    std::string_view bname = std::string_view((const char*)basename.c_str());
    std::string final_name = (const char*)basename.c_str();
    final_name += grid->getName();
    final_name += "_";
    final_name += digits;
    final_name += ".dds";
    std::string final_path = std::filesystem::path(output_dir).append(final_name).string();
    return final_path;
}

std::string getFilepath(const std::u8string& path, const std::u8string& basename, const std::u8string& output_dir, const char* kind)
{
    std::string name = std::filesystem::path(path).stem().string();
    std::string_view digits = std::string_view(name).substr(basename.size());
    std::string_view bname = std::string_view((const char*)basename.c_str());
    std::string final_name = (const char*)basename.c_str();
    final_name += kind;
    final_name += "_";
    final_name += digits;
    final_name += ".dds";
    std::string final_path = std::filesystem::path(output_dir).append(final_name).string();
    return final_path;
}

template<class GridType, class ValueType>
bool saveAsDDS(const std::u8string& path, const std::u8string& basename, const std::u8string& output_dir, const openvdb::GridBase::Ptr grid)
{
    openvdb::CoordBBox volumeActiveAABB = grid->evalActiveVoxelBoundingBox();
    openvdb::Coord activeVoxelDim = grid->evalActiveVoxelDim();
    int32_t width = activeVoxelDim.x() + 1;
    int32_t height = activeVoxelDim.y() + 1;
    int32_t depth = activeVoxelDim.z() + 1;
    ValueType* pixels = new ValueType[width * height * depth];
    ::memset(pixels, 0, sizeof(ValueType) * width * height * depth);
    const GridType* ptr = ((const GridType*)grid.get());
    for(typename GridType::ValueOnCIter iter = ptr->cbeginValueOn(); iter.test(); ++iter) {
        if(!iter.isVoxelValue()) {
            continue;
        }
        const auto& value = *iter;
        openvdb::Coord coord = iter.getCoord();
        coord.x() -= volumeActiveAABB.min().x();
        coord.y() -= volumeActiveAABB.min().y();
        coord.z() -= volumeActiveAABB.min().z();
        assert(0 <= coord.x() && coord.x() < width);
        assert(0 <= coord.y() && coord.y() < height);
        assert(0 <= coord.z() && coord.z() < depth);
        int32_t index = (coord.z() * height + coord.y()) * width + coord.x();
        convert(value, pixels[index]);
    }

    std::string final_path = getFilepath(path, basename, output_dir, grid);
    cppimg::OFStream file;
    if(!file.open(final_path.c_str())) {
        delete[] pixels;
        return false;
    }

    cppimg::DDS::TextureDesc texDesc;
    texDesc.dimension_ = cppimg::DDS::ResourceDimension::Texture3D;
    texDesc.width_ = static_cast<uint32_t>(width);
    texDesc.height_ = static_cast<uint32_t>(height);
    texDesc.depth_ = static_cast<uint32_t>(depth);
    texDesc.mipmapCount_ = 1;
    texDesc.arraySize_ = 1;
    texDesc.format_ = toFormat<ValueType>();
    texDesc.flag_ = cppimg::DDS::ResourceMiscFlag::None;
    texDesc.alphaMode_ = cppimg::DDS::AlphaMode::Straight;
    texDesc.pitch_ = texDesc.calcPitchSize();
    bool result = cppimg::DDS::write(file, texDesc, pixels);
    file.close();
    delete[] pixels;
#ifdef _DEBUG
    if(result) {
        std::cout << "Saved: " << final_path << std::endl;
    } else {
        std::cout << "Failed to save: " << final_path << std::endl;
    }
#endif
    return result;
}

template<class GridType>
bool saveDensityAsDDS(
    const std::u8string& path,
    const std::u8string& basename,
    int32_t countDigits,
    const std::u8string& output_dir,
    int32_t width,
    int32_t height,
    int32_t depth,
    openvdb::GridBase::ConstPtr grid)
{
    openvdb::CoordBBox volumeActiveAABB = grid->evalActiveVoxelBoundingBox();
    openvdb::Coord activeVoxelDim = grid->evalActiveVoxelDim();
    int32_t w = activeVoxelDim.x() + 1;
    int32_t h = activeVoxelDim.y() + 1;
    int32_t d = activeVoxelDim.z() + 1;
    int32_t offset_x = ((std::abs)(w - width) / 2);
    int32_t offset_y = ((std::abs)(h - height) / 2);
    int32_t offset_z = ((std::abs)(d - depth) / 2);
    uint8_t* pixels = new uint8_t[width * height * depth];
    ::memset(pixels, 0, sizeof(uint8_t) * width * height * depth);
    const GridType* ptr = ((const GridType*)grid.get());
    for(typename GridType::ValueOnCIter iter = ptr->cbeginValueOn(); iter.test(); ++iter) {
        if(!iter.isVoxelValue()) {
            continue;
        }
        const auto& value = *iter;
        openvdb::Coord coord = iter.getCoord();
        coord.x() -= volumeActiveAABB.min().x();
        coord.y() -= volumeActiveAABB.min().y();
        coord.z() -= volumeActiveAABB.min().z();
        coord.x() += offset_x;
        coord.y() += offset_y;
        coord.z() += offset_z;
        assert(0 <= coord.x() && coord.x() < width);
        assert(0 <= coord.y() && coord.y() < height);
        assert(0 <= coord.z() && coord.z() < depth);
        int32_t index = (coord.z() * height + coord.y()) * width + coord.x();
        convert(value, pixels[index]);
    }

    std::string final_path = getFilepath(path, basename, output_dir, grid);
    cppimg::OFStream file;
    if(!file.open(final_path.c_str())) {
        delete[] pixels;
        return false;
    }

    cppimg::DDS::TextureDesc texDesc;
    texDesc.dimension_ = cppimg::DDS::ResourceDimension::Texture3D;
    texDesc.width_ = static_cast<uint32_t>(width);
    texDesc.height_ = static_cast<uint32_t>(height);
    texDesc.depth_ = static_cast<uint32_t>(depth);
    texDesc.mipmapCount_ = 1;
    texDesc.arraySize_ = 1;
    texDesc.format_ = toFormat<uint8_t>();
    texDesc.flag_ = cppimg::DDS::ResourceMiscFlag::None;
    texDesc.alphaMode_ = cppimg::DDS::AlphaMode::Straight;
    texDesc.pitch_ = texDesc.calcPitchSize();
    bool result = cppimg::DDS::write(file, texDesc, pixels);
    file.close();
    delete[] pixels;
#ifdef _DEBUG
    if(result) {
        std::cout << "Saved: " << final_path << std::endl;
    } else {
        std::cout << "Failed to save: " << final_path << std::endl;
    }
#endif
    return result;
}

static bool saveAsDDS(
    const std::u8string& path,
    const std::u8string& basename,
    int32_t countDigits,
    const std::u8string& output_dir,
    const openvdb::GridBase::Ptr grid)
{
    if(grid->isType<OpenVDBHalf1Grid>()) {
        return saveAsDDS<OpenVDBHalf1Grid, openvdb::Half>(path, basename, output_dir, grid);

    } else if(grid->isType<OpenVDBFloat1Grid>()) {
        return saveAsDDS<OpenVDBFloat1Grid, float>(path, basename, output_dir, grid);

    } else if(grid->isType<OpenVDBDouble1Grid>()) {
        return saveAsDDS<OpenVDBDouble1Grid, float>(path, basename, output_dir, grid);
    }
    return false;
}

static bool saveDensityAsDDS(
    const std::u8string& path,
    const std::u8string& basename,
    int32_t countDigits,
    const std::u8string& output_dir,
    int32_t width,
    int32_t height,
    int32_t depth,
    openvdb::GridBase::ConstPtr grid_density)
{
    assert(nullptr != grid_density);
    if(grid_density->isType<OpenVDBHalf1Grid>()) {
        return saveDensityAsDDS<OpenVDBHalf1Grid>(path, basename, countDigits, output_dir, width, height, depth, grid_density);

    } else if(grid_density->isType<OpenVDBFloat1Grid>()) {
        return saveDensityAsDDS<OpenVDBFloat1Grid>(path, basename, countDigits, output_dir, width, height, depth, grid_density);

    } else if(grid_density->isType<OpenVDBDouble1Grid>()) {
        return saveDensityAsDDS<OpenVDBDouble1Grid>(path, basename, countDigits, output_dir, width, height, depth, grid_density);
    }
    return false;
}

template<class GridType>
void copyAsFloat16(
    uint16_t* pixels,
    openvdb::GridBase::ConstPtr grid,
    int32_t width,
    int32_t height,
    int32_t depth,
    int32_t offset_x,
    int32_t offset_y,
    int32_t offset_z,
    int32_t element)
{
    assert(0 <= element && element < 1);
    openvdb::CoordBBox volumeActiveAABB = grid->evalActiveVoxelBoundingBox();
    const GridType* ptr = ((const GridType*)grid.get());
    for(typename GridType::ValueOnCIter iter = ptr->cbeginValueOn(); iter.test(); ++iter) {
        if(!iter.isVoxelValue()) {
            continue;
        }
        const auto& value = *iter;
        openvdb::Coord coord = iter.getCoord();
        coord.x() -= volumeActiveAABB.min().x();
        coord.y() -= volumeActiveAABB.min().y();
        coord.z() -= volumeActiveAABB.min().z();
        coord.x() += offset_x;
        coord.y() += offset_y;
        coord.z() += offset_z;
        assert(0 <= coord.x() && coord.x() < width);
        assert(0 <= coord.y() && coord.y() < height);
        assert(0 <= coord.z() && coord.z() < depth);
        int32_t index = (coord.z() * height + coord.y()) * width + coord.x();
        convertTo16(value, pixels[index + element]);
    }
}

template<class GridType>
void copyAsFloat32(
    float* pixels,
    openvdb::GridBase::ConstPtr grid,
    int32_t width,
    int32_t height,
    int32_t depth,
    int32_t offset_x,
    int32_t offset_y,
    int32_t offset_z,
    int32_t element)
{
    assert(0 <= element && element < 1);
    openvdb::CoordBBox volumeActiveAABB = grid->evalActiveVoxelBoundingBox();
    const GridType* ptr = ((const GridType*)grid.get());
    for(typename GridType::ValueOnCIter iter = ptr->cbeginValueOn(); iter.test(); ++iter) {
        if(!iter.isVoxelValue()) {
            continue;
        }
        const auto& value = *iter;
        openvdb::Coord coord = iter.getCoord();
        coord.x() -= volumeActiveAABB.min().x();
        coord.y() -= volumeActiveAABB.min().y();
        coord.z() -= volumeActiveAABB.min().z();
        coord.x() += offset_x;
        coord.y() += offset_y;
        coord.z() += offset_z;
        assert(0 <= coord.x() && coord.x() < width);
        assert(0 <= coord.y() && coord.y() < height);
        assert(0 <= coord.z() && coord.z() < depth);
        int32_t index = (coord.z() * height + coord.y()) * width + coord.x();
        convertTo32(value, pixels[index + element]);
    }
}

static bool saveAsDDS(
    const std::u8string& path,
    const std::u8string& basename,
    int32_t countDigits,
    const std::u8string& output_dir,
    int32_t width,
    int32_t height,
    int32_t depth,
    openvdb::GridBase::ConstPtr grid,
    const char* safix)
{
    assert(nullptr != grid);
    int32_t offset_x = 0;
    int32_t offset_y = 0;
    int32_t offset_z = 0;
    openvdb::Coord activeVoxelDim0;
    {
        activeVoxelDim0 = grid->evalActiveVoxelDim();
        offset_x = ((std::abs)(activeVoxelDim0.x() - width) / 2);
        offset_y = ((std::abs)(activeVoxelDim0.y() - height) / 2);
        offset_z = ((std::abs)(activeVoxelDim0.z() - depth) / 2);
    }
    uint16_t* pixels = new uint16_t[width * height * depth];
    ::memset(pixels, 0, sizeof(uint16_t) * width * height * depth);

    {
        if(grid->isType<OpenVDBHalf1Grid>()) {
            copyAsFloat16<OpenVDBHalf1Grid>(pixels, grid, width, height, depth, offset_x, offset_y, offset_z, 0);

        } else if(grid->isType<OpenVDBFloat1Grid>()) {
            copyAsFloat16<OpenVDBFloat1Grid>(pixels, grid, width, height, depth, offset_x, offset_y, offset_z, 0);

        } else if(grid->isType<OpenVDBDouble1Grid>()) {
            copyAsFloat16<OpenVDBDouble1Grid>(pixels, grid, width, height, depth, offset_x, offset_y, offset_z, 0);
        } else {
            delete[] pixels;
            return false;
        }
    }

    std::string final_path = getFilepath(path, basename, output_dir, safix);
    cppimg::OFStream file;
    if(!file.open(final_path.c_str())) {
        delete[] pixels;
        return false;
    }

    cppimg::DDS::TextureDesc texDesc;
    texDesc.dimension_ = cppimg::DDS::ResourceDimension::Texture3D;
    texDesc.width_ = static_cast<uint32_t>(width);
    texDesc.height_ = static_cast<uint32_t>(height);
    texDesc.depth_ = static_cast<uint32_t>(depth);
    texDesc.mipmapCount_ = 1;
    texDesc.arraySize_ = 1;
    texDesc.format_ = toFormat<openvdb::Half>();
    texDesc.flag_ = cppimg::DDS::ResourceMiscFlag::None;
    texDesc.alphaMode_ = cppimg::DDS::AlphaMode::Straight;
    texDesc.pitch_ = texDesc.calcPitchSize();
    bool result = cppimg::DDS::write(file, texDesc, pixels);
    file.close();
    delete[] pixels;
#ifdef _DEBUG
    if(result) {
        std::cout << "Saved: " << final_path << std::endl;
    } else {
        std::cout << "Failed to save: " << final_path << std::endl;
    }
#endif
    return result;
}

static std::tuple<std::u8string, int32_t> getVDBSequenceBaseFileName(const std::u8string& stem)
{
    assert(0 < stem.size());

    std::u8string_view stem_view(stem);
    int32_t cout_digits = 0;
    for(int64_t i = (int64_t)(stem_view.size() - 1); 0 <= i; --i) {
        if(!std::isdigit(stem[i])) {
            stem_view = stem_view.substr(0, i + 1);
            break;
        }
        ++cout_digits;
    }
    return std::make_tuple(std::u8string(stem_view), cout_digits);
}

static bool valid_name(const std::u8string& name, const std::u8string& base_name, int32_t count_digits)
{
    if(!name.starts_with(base_name)) {
        return false;
    }
    size_t start = base_name.size();
    if(name.size() <= start) {
        return count_digits <= 0 ? true : false;
    }
    for(size_t i = start; i < name.size(); ++i) {
        if(!std::isxdigit(name[i])) {
            return false;
        }
    }
    return true;
}

static std::tuple<std::vector<std::u8string>, std::u8string, int32_t> findOpenVDBSequenceFiles(const std::u8string& source)
{
    std::vector<std::u8string> files;
    std::filesystem::path filepath(source);
    std::u8string stem = filepath.stem().u8string();
    if(stem.length() <= 0 || !std::isdigit(stem[stem.length() - 1])) {
        files.push_back(source);
        return std::make_tuple(files, std::u8string(), 0);
    }
    auto [base_name, count_digits] = getVDBSequenceBaseFileName(stem);

    std::filesystem::path directory = filepath.parent_path();
    for(const std::filesystem::directory_entry& x: std::filesystem::directory_iterator(directory)) {
        if(!x.is_regular_file()) {
            continue;
        }
        std::u8string y = x.path().stem().u8string();
        if(!valid_name(y, base_name, count_digits)) {
            continue;
        }
        files.push_back(x.path().u8string());
    }
    std::sort(files.begin(), files.end());
    return std::make_tuple(files, base_name, count_digits);
}
} // namespace convvdb

int main(int argc, char** argv)
{
    args::ArgumentParser parser("This is a test program.", "This goes after the options.");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag<std::string> option_output(parser, "output", "The output directory", {'o', "output"});
    args::ValueFlag<bool> option_quantize(parser, "quantize", "Quantize output textures", {'q', "quantize"}, true);
    args::Positional<std::string> option_filepath(parser, "filepath", "The path to vdb");
    try {
        parser.ParseCLI(argc, argv);
    } catch(args::Help) {
        std::cerr << parser;
        return 0;
    } catch(args::ParseError e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return -1;
    } catch(args::ValidationError e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return -1;
    }
    if(!option_filepath) {
        std::cerr << parser;
        return -1;
    }
    std::u8string filepath((const char8_t*)args::get(option_filepath).c_str());
    {
        std::filesystem::path path(filepath);
        path = std::filesystem::absolute(path);
        filepath = path.u8string();
    }
    if(!std::filesystem::exists(filepath) || !std::filesystem::is_regular_file(filepath)) {
        std::cerr << parser;
        return -1;
    }
    std::cerr << parser;
    std::u8string output_dir;
    if(option_output) {
        output_dir = (const char8_t*)args::get(option_output).c_str();
        if(!std::filesystem::exists(output_dir)) {
            if(!std::filesystem::create_directories(output_dir)) {
                std::cerr << parser;
                return -1;
            }
        }

        if(!std::filesystem::is_directory(output_dir)) {
            std::cerr << parser;
            return -1;
        }
        output_dir = std::filesystem::absolute(output_dir).u8string();
    } else {
        std::filesystem::path path(filepath);
        output_dir = path.parent_path().u8string();
        if(!std::filesystem::exists(output_dir) || !std::filesystem::is_directory(output_dir)) {
            std::cerr << parser;
            return -1;
        }
    }
    bool quantize = true;
    if(option_quantize) {
        quantize = args::get(option_quantize);
    }
    auto [files, basename, countDigits] = convvdb::findOpenVDBSequenceFiles(filepath);

    // Initialize the OpenVDB library.  This must be called at least
    // once per program and may safely be called multiple times.
    openvdb::initialize();

    for(const std::u8string& f: files) {
        std::ifstream istream((const char*)f.c_str(), std::ios::binary);
        openvdb::io::Stream stream;
        try {
            stream = openvdb::io::Stream(istream, false);
            std::filesystem::path path(f);
            openvdb::GridPtrVecPtr grids = stream.getGrids();
            if(quantize) {
                static const openvdb::Name grid_type("grid_type");
                static const openvdb::Name grid_name("name");
                static const openvdb::Name grid_type_density("density");
                static const openvdb::Name grid_type_emission("flames");
                static const openvdb::Name grid_type_temperature("temperature");

                openvdb::GridBase::ConstPtr grid_density = nullptr;
                openvdb::GridBase::ConstPtr grid_emission = nullptr;
                openvdb::GridBase::ConstPtr grid_temperature = nullptr;
                for(const openvdb::GridBase::Ptr& grid: *grids) {
                    grid->print(std::cout);
                    openvdb::StringMetadata::Ptr metadata = grid->getMetadata<openvdb::StringMetadata>(grid_type);
                    if(nullptr == metadata) {
                        metadata = grid->getMetadata<openvdb::StringMetadata>(grid_name);
                        if(nullptr == metadata) {
                            continue;
                        }
                    }
                    if(grid_type_density == metadata->value()) {
                        grid_density = grid;
                    } else if(grid_type_emission == metadata->value()) {
                        grid_emission = grid;
                    } else if(grid_type_temperature == metadata->value()) {
                        grid_temperature = grid;
                    }
                }
                int32_t width = 0;
                int32_t height = 0;
                int32_t depth = 0;
                if(nullptr != grid_density) {
                    openvdb::Coord activeVoxelDim = grid_density->evalActiveVoxelDim();
                    width = (std::max)(width, activeVoxelDim.x());
                    height = (std::max)(height, activeVoxelDim.y());
                    depth = (std::max)(depth, activeVoxelDim.z());
                }
                if(nullptr != grid_emission) {
                    openvdb::Coord activeVoxelDim = grid_emission->evalActiveVoxelDim();
                    width = (std::max)(width, activeVoxelDim.x());
                    height = (std::max)(height, activeVoxelDim.y());
                    depth = (std::max)(depth, activeVoxelDim.z());
                }
                if(nullptr != grid_temperature) {
                    openvdb::Coord activeVoxelDim = grid_temperature->evalActiveVoxelDim();
                    width = (std::max)(width, activeVoxelDim.x());
                    height = (std::max)(height, activeVoxelDim.y());
                    depth = (std::max)(depth, activeVoxelDim.z());
                }
                width += 1;
                height += 1;
                depth += 1;

                if(nullptr != grid_density) {
                    convvdb::saveDensityAsDDS(f, basename, countDigits, output_dir, width, height, depth, grid_density);
                }
                if(nullptr != grid_emission) {
                    convvdb::saveAsDDS(f, basename, countDigits, output_dir, width, height, depth, grid_emission, "emission");
                }
                if(nullptr != grid_temperature) {
                    convvdb::saveAsDDS(f, basename, countDigits, output_dir, width, height, depth, grid_temperature, "temperature");
                }

            } else {
                for(const openvdb::GridBase::Ptr& grid: *grids) {
                    convvdb::saveAsDDS(f, basename, countDigits, output_dir, grid);
                }
            }
        } catch(const openvdb::Exception& /*Exception*/) {
        }

        istream.close();
    }
    return 0;
}

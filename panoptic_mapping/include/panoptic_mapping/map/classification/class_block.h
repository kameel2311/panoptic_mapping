#ifndef PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_BLOCK_H_
#define PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_BLOCK_H_

#include <memory>
#include <utility>

#include <voxblox/core/block.h>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/classification/class_voxel.h"

namespace panoptic_mapping {

/**
 * @brief General interface for classification voxel blocks. Wraps the voxblox
 * block to allow substituting different classification layer types.
 */
class ClassBlock {
 public:
  typedef std::shared_ptr<ClassBlock> Ptr;
  typedef std::shared_ptr<const ClassBlock> ConstPtr;

  virtual ~ClassBlock() = default;

  // Implements all voxel access interfaces.
  virtual const ClassVoxel& getVoxelByLinearIndex(size_t index) const = 0;
  virtual const ClassVoxel& getVoxelByVoxelIndex(
      const VoxelIndex& index) const = 0;
  virtual const ClassVoxel& getVoxelByCoordinates(
      const Point& coords) const = 0;
  virtual ClassVoxel& getVoxelByCoordinates(const Point& coords) = 0;
  virtual ClassVoxel* getVoxelPtrByCoordinates(const Point& coords) = 0;
  virtual const ClassVoxel* getVoxelPtrByCoordinates(
      const Point& coords) const = 0;
  virtual ClassVoxel& getVoxelByLinearIndex(size_t index) = 0;
  virtual ClassVoxel& getVoxelByVoxelIndex(const VoxelIndex& index) = 0;
  virtual ClassVoxelType getVoxelType() const = 0;
};

/**
 * @brief Implements all the interface fucntions to wrap a block of VoxelT
 * voxels.
 *
 * @tparam VoxelT Voxel type contained in the wrapped block. Needs to inherit
 * from ClassVoxel.
 */
template <typename VoxelT>
class ClassBlockImpl : public ClassBlock {
 public:
  explicit ClassBlockImpl(std::shared_ptr<voxblox::Block<VoxelT>> block)
      : block_(std::move(block)) {}

  const ClassVoxel& getVoxelByLinearIndex(size_t index) const override {
    return block_->getVoxelByLinearIndex(index);
  }
  const ClassVoxel& getVoxelByVoxelIndex(
      const VoxelIndex& index) const override {
    return block_->getVoxelByVoxelIndex(index);
  }
  const ClassVoxel& getVoxelByCoordinates(const Point& coords) const override {
    return block_->getVoxelByCoordinates(coords);
  }
  ClassVoxel& getVoxelByCoordinates(const Point& coords) override {
    return block_->getVoxelByCoordinates(coords);
  }
  ClassVoxel* getVoxelPtrByCoordinates(const Point& coords) override {
    return block_->getVoxelPtrByCoordinates(coords);
  }
  const ClassVoxel* getVoxelPtrByCoordinates(
      const Point& coords) const override {
    return block_->getVoxelPtrByCoordinates(coords);
  }
  ClassVoxel& getVoxelByLinearIndex(size_t index) override {
    return block_->getVoxelByLinearIndex(index);
  }
  ClassVoxel& getVoxelByVoxelIndex(const VoxelIndex& index) override {
    return block_->getVoxelByVoxelIndex(index);
  }
  ClassVoxelType getVoxelType() const override {
    return reinterpret_cast<const ClassVoxel&>(block_->getVoxelByLinearIndex(0))
        .getVoxelType();
  }

  // Exposes the actual block if the type is known.
  voxblox::Block<VoxelT>& getBlock() { return *block_; }
  const voxblox::Block<VoxelT>& getBlock() const { return *block_; }

 private:
  const std::shared_ptr<voxblox::Block<VoxelT>> block_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_CLASSIFICATION_CLASS_BLOCK_H_

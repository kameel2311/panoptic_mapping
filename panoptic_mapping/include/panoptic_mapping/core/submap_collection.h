#ifndef PANOPTIC_MAPPING_CORE_SUBMAP_COLLECTION_H_
#define PANOPTIC_MAPPING_CORE_SUBMAP_COLLECTION_H_

#include <memory>
#include <vector>
#include <map>

#include "panoptic_mapping/core/common.h"
#include "panoptic_mapping/core/submap.h"

namespace panoptic_mapping {

/***
 * This class contains and manages access to all submaps.
 */
class SubmapCollection {
 public:
  SubmapCollection() = default;
  virtual ~SubmapCollection() = default;

  // iterator over submaps
  std::vector<Submap>::iterator begin() {return submaps_.begin();}
  std::vector<Submap>::iterator end() {return submaps_.end();}

  // modify the collection
  bool addSubmap(const Submap &submap);
  bool removeSubmap(int id);      // removes all submaps of this id
  bool changeSubmapID(int id_old, int id_new);
  bool submapIdExists(int id);    // check whether id exists
  Submap& getSubmap(int id);      // this assumes that the id exists
  void clear();

  // accessors
  size_t size() { return submaps_.size(); }

 private:
  std::vector<Submap> submaps_;
  std::unordered_map<int, size_t> id_to_index_;
};

} // namespace panoptic_mapping

#endif //PANOPTIC_MAPPING_CORE_SUBMAP_COLLECTION_H_

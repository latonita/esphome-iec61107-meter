#include "object_locker.h"

namespace esphome {
namespace iec61107 {

std::vector<void *> AnyObjectLocker::locked_objects_(5);
Mutex AnyObjectLocker::lock_;

};  // namespace iec61107
};  // namespace esphome

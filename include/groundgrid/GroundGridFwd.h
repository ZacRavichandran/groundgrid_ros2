#pragma once

#include <memory>

namespace groundgrid {

/** Forward declaration of the GroundGrid class with type aliases.
 **
 ** If the GroundGrid class is pointed from another class without using its methods,
 ** then only this forward declaration is needed (saves compile time).
 **
 */
class GroundGrid;

using GroundGridPtr = std::shared_ptr<GroundGrid>;
using ConstGroundGridPtr = std::shared_ptr<const GroundGrid>;
}

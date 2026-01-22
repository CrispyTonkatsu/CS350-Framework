#include "Application.hpp"
#include "BspTree.hpp"
#include "Components.hpp"
#include "DebugDraw.hpp"
#include "DynamicAabbTree.hpp"
#include "Geometry.hpp"
#include "Math/Utilities.hpp"
#include "Precompiled.hpp"
#include "Shapes.hpp"
#include "SimpleNSquared.hpp"
#include "UnitTests.hpp"

void InitializeAssignment4Tests()
{
    mTestFns.push_back(AssignmentUnitTestList());
    AssignmentUnitTestList& list = mTestFns.back();
}

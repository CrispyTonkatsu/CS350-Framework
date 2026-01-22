#include "Application.hpp"
#include "Components.hpp"
#include "DebugDraw.hpp"
#include "DynamicAabbTree.hpp"
#include "Geometry.hpp"
#include "Gjk.hpp"
#include "Math/Utilities.hpp"
#include "Precompiled.hpp"
#include "Shapes.hpp"
#include "SimpleNSquared.hpp"
#include "UnitTests.hpp"

void InitializeAssignment5Tests()
{
    mTestFns.push_back(AssignmentUnitTestList());
    AssignmentUnitTestList& list = mTestFns.back();
}

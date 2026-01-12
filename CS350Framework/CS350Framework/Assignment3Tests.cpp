#include "Precompiled.hpp"
#include "Application.hpp"
#include "Shapes.hpp"
#include "Geometry.hpp"
#include "Math/Utilities.hpp"
#include "DebugDraw.hpp"
#include "Components.hpp"
#include "SimpleNSquared.hpp"
#include "UnitTests.hpp"
#include "DynamicAabbTree.hpp"

void InitializeAssignment3Tests()
{
  mTestFns.push_back(AssignmentUnitTestList());
  AssignmentUnitTestList& list = mTestFns[2];
}

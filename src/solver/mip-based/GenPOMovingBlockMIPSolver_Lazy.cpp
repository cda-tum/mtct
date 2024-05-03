#include "EOMHelper.hpp"
#include "MultiArray.hpp"
#include "gurobi_c++.h"
#include "solver/mip-based/GenPOMovingBlockMIPSolver.hpp"
#include "solver/mip-based/GeneralMIPSolver.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)

void cda_rail::solver::mip_based::GenPOMovingBlockMIPSolver::LazyCallback::
    callback() {
  try {
    if (where == GRB_CB_MESSAGE) {
      MessageCallback::callback();
    } else if (where == GRB_CB_MIPSOL) {
      PLOGD << "TODO: Implement callback" << std::endl;
    }
  } catch (GRBException e) {
    PLOGE << "Error number: " << e.getErrorCode() << std::endl;
    PLOGE << e.getMessage() << std::endl;
  } catch (...) {
    PLOGE << "Error during callback" << std::endl;
  }
}

// NOLINTEND(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-pro-bounds-array-to-pointer-decay,performance-inefficient-string-concatenation)

//
// Created by Dave Durbin on 18/5/20.
//

#include "AbstractOptimiser.h"

#include <utility>


AbstractOptimiser::AbstractOptimiser(Properties properties) : m_properties(std::move(properties)),
                                                              m_state{UNINITIALISED} {
}

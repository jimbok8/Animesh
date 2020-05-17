//
// Created by Dave Durbin on 18/5/20.
//

#ifndef ANIMESH_ABSTRACTOPTIMISER_H
#define ANIMESH_ABSTRACTOPTIMISER_H

#include <Properties/Properties.h>

class AbstractOptimiser {
public:
    explicit AbstractOptimiser(Properties properties);

    /**
     * State of the opitimiser.
     */
    enum OptimisationState {
        UNINITIALISED,
        INITIALISED,
        OPTIMISING,
        ENDING_OPTIMISATION
    } m_state;

private:
    Properties m_properties;
};

#endif //ANIMESH_ABSTRACTOPTIMISER_H

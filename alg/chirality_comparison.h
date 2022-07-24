/* Copyright (C) 2022 Daniel Warner.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

// Defines the particle system and composing particles for the General
// Formation Algorithm as alluded to in 'An Algorithmic Framework for Shape
// Formation Problems in Self-Organizing Particle Systems'
// [arxiv.org/abs/1504.00744].
//
// Extension of the shape formation algorithms in order to make them tolerant to the failure of particles.
//
// mode == "h" --> hexagon formation
// mode == "s" --> square formation
// mode == "t1" --> vertex triangle formation
// mode == "t2" --> center triangle formation
// mode == "l" --> line formation

#ifndef AMOEBOTSIM_ALG_CHIRALITY_COMPARISON_H_
#define AMOEBOTSIM_ALG_CHIRALITY_COMPARISON_H_

#include <set>

#include <QString>

#include <bits/stl_list.h>

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

class ChiralityComparisonParticle : public AmoebotParticle {
public:
    enum class State {
        Idle,
        Done,
    };

    // Constructs a new particle with a node position for its head, a global
    // compass direction from its head to its tail (-1 if contracted), an offset
    // for its local compass, a system which it belongs to, and an initial state.
    ChiralityComparisonParticle(const Node head, const int globalTailDir,
                                        const int orientation, AmoebotSystem& system,
                                        State state);

    // Executes one particle activation.
    virtual void activate();

    // Functions for altering a particle's cosmetic appearance.
    // particleColor returns the color to be used for the particle.
    // headMarkColor (respectively, tailMarkColor) returns the color to be used for the ring
    // drawn around the head (respectively, tail) node. Tail color is not shown
    // when the particle is contracted.
    virtual int particleColor() const;
    virtual int headMarkColor() const;
    virtual int tailMarkColor() const;

    // Returns the string to be displayed when this particle is inspected; used
    // to snapshot the current values of this particle's memory at runtime.
    virtual QString inspectionText() const;

    // Returns the safeFlags array associated with the
    // particle to draw the safeFlags of the particle.
    virtual std::array<std::array<std::array<VisFlagStates, 2>, 2>, 6> getSafeFlags() const;

    // Gets a reference to the neighboring particle incident to the specified port
    // label. Crashes if no such particle exists at this label; consider using
    // hasNbrAtLabel() first if unsure.
    ChiralityComparisonParticle& nbrAtLabel(int label) const;

    // Returns the label of the first port incident to a neighboring particle in
    // any of the specified states, starting at the (optionally) specified label
    // and continuing clockwise.
    int labelOfFirstNbrInState(std::initializer_list<State> states,
                               int startLabel = 0, bool ignoreErrorParticles = true) const;

    // Checks whether this particle has a neighbor in any of the given states.
    bool hasNbrInState(std::initializer_list<State> states) const;

    // Checks whether this particle's state is one of the given states.
    bool isInState(std::initializer_list<State> states) const;

protected:
    // General state of this (error or non-error) particle.
    State state;

    // hasInvalidated array of this (error) particle:
    // A particle that is in safeState Safe sends "invalidate" messages to followers
    // who could be potential parents. The hasInvalidated array stores which neighbours
    // have been contacted so far. This is visualized by small circles
    // in dark (instead of light) green in the corresponding directions.
    std::array<bool, 6> hasCompared;

    bool isBoundaryParticle;

private:
    friend class ChiralityComparisonSystem;
};

class ChiralityComparisonSystem : public AmoebotSystem  {
public:
    // Constructs a system of ChiralityComparisonParticles with an optionally specified
    // size (#particles), hole probability, and shape to form. holeProb in [0,1]
    // controls how "spread out" the system is; closer to 0 is more compressed,
    // closer to 1 is more expanded. The current shapes accepted are...
    //   "h"  --> hexagon
    //   "s"  --> square
    //   "t1" --> vertex triangle
    //   "t2" --> center triangle
    //   "l"  --> line
    ChiralityComparisonSystem(int numParticles = 200, double holeProb = 0.2,
                                      QString mode = "h");

    // Checks whether or not the system's run of the ShapeFormation formation
    // algorithm has terminated (all particles finished, i.e. apart from Seed all particles in state Retired).
    bool hasTerminated() const override;

    // Returns a set of strings containing the current accepted modes of
    // Shapeformation.
    static std::set<QString> getAcceptedModes();
};

#endif  // AMOEBOTSIM_ALG_CHIRALITY_COMPARISON_H_

/* Copyright (C) 2021 Daniel Warner.
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

#ifndef AMOEBOTSIM_ALG_SHAPEFORMATION_FAULT_TOLERANT_H_
#define AMOEBOTSIM_ALG_SHAPEFORMATION_FAULT_TOLERANT_H_

#include <set>

#include <QString>

#include <bits/stl_list.h>

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

class ShapeFormationFaultTolerantParticle : public AmoebotParticle {
public:
    enum class State {
        Seed,
        Idle,
        Follower,
        Root,
        Retired,
        Crashed,  // When a particle crashes/is scheduled to crash, its state changes to the Crashed state.
        Error
    };

    enum class PathToRootState {
        Valid,
        Invalid,
        Invalidate
    };

    enum class SafeState : int {
        Undetermined = 0,
        Unsafe = 1,
        Safe = 2
    };

    // Constructs a new particle with a node position for its head, a global
    // compass direction from its head to its tail (-1 if contracted), an offset
    // for its local compass, a system which it belongs to, an initial state, and
    // a string to determine what shape to form.
    ShapeFormationFaultTolerantParticle(const Node head, const int globalTailDir,
                                        const int orientation, AmoebotSystem& system,
                                        State state, const QString mode);

    // Causes the particle to crash. (i.e. change the state of the particle to the state Crashed)
    virtual void crash();

    // Executes one particle activation.
    virtual void activate();

    // Determine whether the particle is in an error state (here: state Crashed or state Error).
    virtual bool isErrorParticle() const;

    // Determine whether the particle has a neighbour in an error state (using isErrorParticle)
    virtual bool hasErrorNbr() const;

    // Functions for altering a particle's cosmetic appearance.
    // particleColor returns the color to be used for the particle.
    // particleCenterColor returns the color to be used as a small marking
    // in the center of the particle (here: visualize a "noted" invalidate
    // for a particle in state Error which will be propagated as soon as the particle becomes a follower).
    // headMarkColor (respectively, tailMarkColor) returns the color to be used for the ring
    // drawn around the head (respectively, tail) node. Tail color is not shown
    // when the particle is contracted. headMarkDir returns the label of the port
    // on which the black head marker is drawn.
    virtual int particleColor() const;
    virtual int particleCenterColor() const;
    virtual int headMarkColor() const;
    virtual int headMarkDir() const;
    virtual int tailMarkColor() const;

    // Returns the string to be displayed when this particle is inspected; used
    // to snapshot the current values of this particle's memory at runtime.
    virtual QString inspectionText() const;

    // Returns the _borderColors array associated with the
    // particle to draw the boundaries of the hexagon layers.
    virtual std::array<int, 18> borderColors() const;
    // Updates the _borderColors array.
    void updateBorderColors();

    // Returns the safeFlags array associated with the
    // particle to draw the safeFlags of the particle.
    virtual std::array<std::array<std::array<VisFlagStates, 2>, 2>, 6> getSafeFlags() const;

    // Gets a reference to the neighboring particle incident to the specified port
    // label. Crashes if no such particle exists at this label; consider using
    // hasNbrAtLabel() first if unsure.
    ShapeFormationFaultTolerantParticle& nbrAtLabel(int label) const;

    // Returns the label of the first port incident to a neighboring particle in
    // any of the specified states, starting at the (optionally) specified label
    // and continuing clockwise.
    int labelOfFirstNbrInState(std::initializer_list<State> states,
                               int startLabel = 0, bool ignoreErrorParticles = true) const;

    // Checks whether this particle has a neighbor in any of the given states.
    bool hasNbrInState(std::initializer_list<State> states) const;

    // Checks whether this particle's state is one of the given states.
    bool isInState(std::initializer_list<State> states) const;

    // Returns the label of the port incident to a neighbor which is finished and
    // pointing at this particle's position as the next one to be filled; returns
    // -1 if such a neighbor does not exist.
    int constructionReceiveDir() const;

    // Checks whether this particle is occupying the next position to be filled and
    // therefore may become Retired.
    bool canRetire() const;

    // A particle is finished if it is in state Retired or Seed.
    bool isFinished() const;

    // Function updateConstructionDir tries to set this particle's constructionDir
    // to point at the next position to be filled as it is finishing.
    // Returns true on success.
    bool updateConstructionDir();

    // Function computeMoveLabel tries to compute this particle's label
    // to traverse the current surface of the forming shape counter-clockwise when it is a root.
    // Returns -1 if label could not be computed successfully.
    int computeMoveLabel() const;
    // Function updateMoveDir tries to update this particle's moveDir
    // using the function computeMoveLabel. Returns true on success.
    bool updateMoveDir();

    // Checks whether this (expanded) particle has an immediate child
    // in the spanning tree following its tail.
    bool hasTailFollower() const;

    // Checks whether this (expanded) particle has a tail follower or
    // an idle or error neighbour which is adjacent to its tail.
    bool hasBlockingTailNbr() const;

    // Fault-tolerant method (as part of the fault-tolerant shape formation protocol)
    // that may perform movements:
    void performMovement();

    // Fault-tolerant methods (as part of the fault-tolerant shape formation protocol)
    // that may change the state of the particle but do not perform any movements:
    bool tryToBecomeRetired_Hexagon();
    bool tryToBecomeRoot_Hexagon();
    bool tryToBecomeFollower();
    bool tryFollowerRecoveryByPropagation();

    // Update the flags of the particle.
    void updateFlags();

    // Propagate Invalidate
    void propagateInvalidate();

    // Propagate Valid
    void propagateValid();

protected:
    // General state of this (error or non-error) particle.
    State state;

    // mode stores a string that specifies which shape (hexagon, triangle, etc.) is formed.
    QString mode;
    // turnSignal is used for the square and vertex triangle shape formation algorithms.
    int turnSignal;

    // Shape formation specific variables of this (non-error) particle:
    int constructionDir;
    int moveDir;
    int followDir;

    // State of this (error or non-error) particle as node(s) on a path upwards to a root particle.
    //
    // All particles are initially Valid. Error particles are initialized Invalid.
    // Seed, Retired, Root and Idle particles are always Valid.
    // Follower particles can be in state Valid, Invalid or Invalidate.
    // Error particles can be in state Invalid or Invalidate.
    //
    // The pathToRoot state is only propagated by non-error particles:
    // - Follower in state Invalidate: Becomes Invalid and propagates upwards:
    //   If it has a follower or Error particle as parent, the parent's pathToRoot state becomes Invalidate.
    // - Follower in state Invalid: Remains invalid and no propagation.
    // - Follower in state Valid or Root: Remains Valid and propagates downwards:
    //   If it has an invalid follower child (an invalid neighbour follower pointing at it), the child becomes valid.
    //   In particular, if the child is in state Invalidate, the child does not change its pathToRoot state (Invalidate "beats" Valid).
    //
    // Basic idea: By propagating "Invalidate" upwards to the root, all particles on the path upwards become "Invalid" (=> Safety).
    // After "Invalidate" is finally "consumed" by the root, the status "Valid" is propagated
    // from the root downwards to the leaves (=> Liveness), whereby the validation process might again be interrupted by an ascending "Invalidate".
    PathToRootState pathToRootState;

    // safeFlags of this (error particle):
    // safeFlags are used to determine how many rounds the current value of safeState is based on.
    std::array<std::array<std::array<SafeState, 2>, 2>, 6> safeFlags;

    // safeState of this (error) particle:
    // A particle p in state Error may become a follower if all of the following conditions are met:
    // 1) p.safeState == Safe
    // 2) p has a follower neighbour in pathToRootState Valid which was previously invalidated by p, or p has a root neighbour.
    SafeState safeState;

    // hasInvalidated array of this (error) particle:
    // A particle that is in safeState Safe sends "invalidate" messages to followers
    // who could be potential parents. The hasInvalidated array stores which neighbours
    // have been contacted so far. This is visualized by small circles
    // in dark (instead of light) green in the corresponding directions.
    std::array<bool, 10> hasInvalidated;

    // _borderColorsSet and _borderColors are used to draw the boundaries of the hexagon layers.
    bool _borderColorsSet;
    std::array<int, 18> _borderColors;

private:
    friend class ShapeFormationFaultTolerantSystem;
};

class ShapeFormationFaultTolerantSystem : public AmoebotSystem  {
public:
    // Constructs a system of ShapeFormationFaultTolerantParticles with an optionally specified
    // size (#particles), hole probability, and shape to form. holeProb in [0,1]
    // controls how "spread out" the system is; closer to 0 is more compressed,
    // closer to 1 is more expanded. The current shapes accepted are...
    //   "h"  --> hexagon
    //   "s"  --> square
    //   "t1" --> vertex triangle
    //   "t2" --> center triangle
    //   "l"  --> line
    ShapeFormationFaultTolerantSystem(int numParticles = 200, double holeProb = 0.2,
                                      QString mode = "h");

    // Checks whether or not the system's run of the ShapeFormation formation
    // algorithm has terminated (all particles finished, i.e. apart from Seed all particles in state Retired).
    bool hasTerminated() const override;

    // nuke causes all particles except the Seed particle to crash.
    void nuke() override;

    // Returns a set of strings containing the current accepted modes of
    // Shapeformation.
    static std::set<QString> getAcceptedModes();
};

class NotImplementedException : public std::logic_error
{
private:

    std::string _text;

public:

    NotImplementedException()
        :
          NotImplementedException("Not implememented", __FUNCTION__)
    {
    }

    NotImplementedException(const char* message)
        :
          NotImplementedException(message, __FUNCTION__)
    {
    }

    NotImplementedException(const char* message, const char* function)
        :
          std::logic_error("Not implemented")
    {
        _text = "Not implemented : ";
        _text += message;
        _text += " : ";
        _text += function;
    };

    virtual const char *what() const throw()
    {
        return _text.c_str();
    }
};

#endif  // AMOEBOTSIM_ALG_SHAPEFORMATION_FAULT_TOLERANT_H_

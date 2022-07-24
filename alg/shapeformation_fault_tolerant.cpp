/* Copyright (C) 2021 Daniel Warner.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "alg/shapeformation_fault_tolerant.h"

#include <QtGlobal>

ShapeFormationFaultTolerantParticle::ShapeFormationFaultTolerantParticle(const Node head,
                                                                         const int globalTailDir,
                                                                         const int orientation,
                                                                         AmoebotSystem& system,
                                                                         State state, const QString mode)
    : AmoebotParticle(head, globalTailDir, orientation, system),
      state(state),
      mode(mode),
      constructionDir(-1),
      moveDir(-1),
      followDir(-1),
      pathToRootState(PathToRootState::Valid),
      safeFlags({}),
      safeState(SafeState::Undetermined),
      hasInvalidated({}),
      _borderColorsSet(false)
{
    _borderColors.fill(-1);

    if (state == State::Seed) {
        constructionDir = 0;
    }
}

bool ShapeFormationFaultTolerantParticle::isErrorParticle() const {
    return isInState({State::Crashed, State::Error});
}

bool ShapeFormationFaultTolerantParticle::hasErrorNbr() const {
    auto prop = [&](const ShapeFormationFaultTolerantParticle& p) {
        return p.isErrorParticle();
    };

    return labelOfFirstNbrWithProperty<ShapeFormationFaultTolerantParticle>(prop) != -1;
}

void ShapeFormationFaultTolerantParticle::crash() {
    // Model assumption: The Seed particle does not crash.
    if (state != State::Seed) {
        state = State::Crashed;
    }
}

void ShapeFormationFaultTolerantParticle::activate() {
    // finished particles (i.e. Seed or Retired) do nothing
    if (isFinished()) {
        return;
    }

    // Initialization of crashed particle after crash ("wake up"):
    if (state == State::Crashed) {
        state = State::Error;

        pathToRootState = PathToRootState::Invalid;

        safeFlags = {};
        safeState = SafeState::Undetermined;

        hasInvalidated = {};

        return; // only relevant for step by step visualization, but not for the correctness of the algorithm.
    }

    // Wait until all neighbours in state Crashed are "awake", i.e. in state Error, and therefore initialized.
    if (hasNbrInState({State::Crashed})) {
        return; // particles with neighbours in state Crashed do nothing
    }

    // Update safeState for error particle
    if (state == State::Error) {
        updateFlags();
    }

    // Try state change:
    if (canRetire()) {
        if (isInState({State::Error, State::Idle, State::Follower, State::Root})) {
            if (tryToBecomeRetired_Hexagon()) {
                pathToRootState = PathToRootState::Valid;
                updateBorderColors(); // only necessary for visualization
                return; // particle has become Retired and therefore does not do anything more.
            }
        }
    } else { // if (!canRetire())
        // It can happen that a particle could in principle retire (because a retired particle is pointing at it),
        // but its retireDir can not be determined due to failed particles, so that tryToBecomeRetired is unsuccessful.
        // This branch is not executed if the particle can in principle retire, i.e. if canRetire returns false.
        // This ensures that in that case the particle can only change its state to Retired.

        if (isInState({State::Error, State::Idle, State::Follower})) {
            if (tryToBecomeRoot_Hexagon()) {
                pathToRootState = PathToRootState::Valid;
                return; // only relevant for step by step visualization, but not for the correctness of the algorithm.
            }
        }

        if (state == State::Idle) {
            if (tryToBecomeFollower()) {
                return; // only relevant for step by step visualization, but not for the correctness of the algorithm.
            }
        }

        if (state == State::Error) {
            if (tryFollowerRecoveryByPropagation()) {
                return; // only relevant for step by step visualization, but not for the correctness of the algorithm.
            }
        }
    }

    // Propagation of pathToRootState. Propagation must be done before performing movement.
    if (state == State::Follower && pathToRootState == PathToRootState::Invalidate) {
        pathToRootState = PathToRootState::Invalid;
        propagateInvalidate();
    } else if (state == State::Root || (state == State::Follower && pathToRootState == PathToRootState::Valid)) {
        propagateValid();
    }

    // Movement
    if (isInState({State::Follower, State::Root})) {
        performMovement();
    }
}

void ShapeFormationFaultTolerantParticle::performMovement() {
    if (isExpanded()) { // Movement of expanded follower/root particles:
        if (isInState({State::Follower, State::Root})) {
            if (!hasBlockingTailNbr()) {
                contractTail();
            }
        } else {
            Q_ASSERT(false);
        }
    } else { // Movement of contracted follower/root particles:
        if (state == State::Follower) {
            if (hasTailAtLabel(followDir)) {
                auto& nbr = nbrAtLabel(followDir);
                if (nbr.isInState({State::Follower, State::Root}) ) { // make sure to not push an expanded error neighbour
                    if (nbr.pathToRootState != PathToRootState::Invalidate) { // ensure that Invalidate does not propagate downwards
                        int nbrContractionDir = nbrDirToDir(nbr, (nbr.tailDir() + 3) % 6);
                        push(followDir);
                        followDir = nbrContractionDir;

                        if (nbr.state == State::Root) {
                            nbr.updateMoveDir(); // only relevant for up-to-date visualization, but not for the correctness of the algorithm.
                        }

                        // This (now expanded) particle p has now occupied that node with its head,
                        // which the (previously expanded) neighbor q it pushed had occupied with its tail.
                        // The paths to a root that previously ran over the tail of q now run over the head of p.
                        // Therefore and since propagation has already taken place, p takes on the pathToRootState of q.
                        // This ensures the correctness of the propagation protocol
                        // used to repair followers and furthermore does not restrict the movement of particles.
                        pathToRootState = nbr.pathToRootState;
                    }
                }
            }
        } else if (state == State::Root) {
            if (updateMoveDir()) {
                if (!hasNbrAtLabel(moveDir)) {
                    expand(moveDir);
                } else if (hasTailAtLabel(moveDir)) {
                    // since updateMoveDir() succeeded, the expanded neighbour that has its tail at moveDir must be a non-error particle.
                    // (more precisely: the expanded neighbour must be a root particle).
                    // Therefore this particle may push into that (root) neighbour.
                    auto& nbr = nbrAtLabel(moveDir);
                    push(moveDir);

                    nbr.updateMoveDir(); // only relevant for up-to-date visualization, but not for the correctness of the algorithm.
                }
            }
        }
    }

    if (state == State::Root) {
        updateMoveDir(); // only relevant for an up-to-date visualization, but not for the correctness of the algorithm.
    }
}

bool ShapeFormationFaultTolerantParticle::tryToBecomeRetired_Hexagon() {
    // Protocol for locally building up the hexagon retired structure:

    // By q denote the current particle in state Error.

    // Case 1: q is contracted and has a finished (state Seed or Retired) neighbour p with constructionDir set to its position
    if (canRetire()) {
        constructionDir = constructionReceiveDir();
        if (nbrAtLabel(constructionDir).state == State::Seed) { // Case 1.1: p is in state Seed
            state = State::Retired;
            constructionDir = (constructionDir + 1) % 6;
            return true;
        } else { // Case 1.2: p is in state Retired
            if (!hasNbrInState({State::Seed})) { // Case 1.2.2: q does not have the Seed as neighbour.
                int innerDir = (constructionDir + 1) % 6;
                if (hasNbrAtLabel(innerDir)) {
                    ShapeFormationFaultTolerantParticle& nbr = nbrAtLabel(innerDir);
                    if (nbr.state == State::Retired) {
                        if (nbrDirToDir(nbr, nbr.constructionDir) == innerDir) { // exception of the rule
                            state = State::Retired;
                            constructionDir = (constructionDir + 2) % 6;
                            return true;
                        } else {
                            state = State::Retired;
                            constructionDir = nbrDirToDir(nbr, nbr.constructionDir);
                            return true;
                        }
                    }
                }
            } else { // Case 1.2.1: q has the Seed as neighbour.
                int seedLabel = labelOfFirstNbrInState({State::Seed});
                ShapeFormationFaultTolerantParticle& nbr = nbrAtLabel(seedLabel);
                int position = (9 + nbr.constructionDir - dirToNbrDir(nbr, seedLabel)) % 6; // clockwise index of q with respect to seed
                if (position == 5) {
                    position = 4;
                }
                state = State::Retired;
                constructionDir = nbrDirToDir(nbr, (6 + nbr.constructionDir - 2 - position) % 6);
                return true;
            }
        }
    }

    return false;
}

bool ShapeFormationFaultTolerantParticle::tryToBecomeRoot_Hexagon() {
    // Protocol for local recovery of root particles for shape formation with a centered Seed particle
    // (i.e., the Retired structure is formed in clockwise direction with a centered Seed particle,
    // and the Root particles move in counter-clockwise direction around the Retired structure):

    // By p denote the current particle in state Error.

    // Case: p is contracted, has a finished (state Seed or Retired) neighbour q,
    // but no finished (state Seed or Retired) neighbour with constructionDir set to its position.
    //
    // From particle p's point of view, starting with the label that points to q,
    // let r be the in clockwise direction first non-Retired particle (or empty node) connected to p,
    // and let s be the predecessor of r in this order.
    //
    // If r is not a particle in an error state, then p may become a Root particle
    // (with moveDir set in the direction of r) with one exception
    // (which is only relevant if s is currently the outermost retired particle):
    // Here it must be checked on the basis of s whether p is outside or inside the retired structure.
    //
    // Case: p is expanded and has a finished (state Seed or Retired) neighbour.
    // Then the label can be computed analogously to the contracted case.
    // Additional care has to be taken, to handle a cycle of failed particles (see below).

    if (isContracted() && !canRetire()) {
        int labelNbrR = computeMoveLabel();

        if (labelNbrR != -1) {
            int labelNbrS = (labelNbrR + 1) % 6;
            ShapeFormationFaultTolerantParticle& nbrS = nbrAtLabel(labelNbrS);
            // Check on the basis of s whether p is outside the retired structure:
            if (nbrS.state == State::Seed || (6 + nbrDirToDir(nbrS, nbrS.constructionDir) - labelNbrS) % 6 <= 2) {
                moveDir = labelToDir(labelNbrR);
                state = State::Root;
                return true;
            }
        }
    } else if (isExpanded()) {
        if (hasNbrInState({State::Seed, State::Retired})) { // check whether particle has a finished neighbour
            if (updateMoveDir()) {
                state = State::Root;
                return true;
            } else { // handle cycle of failed particles around retired structure where this (expanded) particle has a retired particle pointing at it.
                auto prop = [&](const ShapeFormationFaultTolerantParticle& p) {
                    return p.isFinished() && pointsAtMe(p, p.constructionDir);
                };

                int label = labelOfFirstNbrWithProperty<ShapeFormationFaultTolerantParticle>(prop, 0);
                if (label != -1) {
                    int nextLabel = (label + 9) % 10;
                    if (nbrNodeReachedViaLabel(label) == nbrNodeReachedViaLabel(nextLabel)) {
                        nextLabel = (nextLabel + 9) % 10;
                    }
                    makeHeadLabel(nextLabel);
                    moveDir = labelToDir(nextLabel);
                    state = State::Root;
                    return true;
                }
            }
        }
    }

    return false;
}

bool ShapeFormationFaultTolerantParticle::tryToBecomeFollower() {
    if (state == State::Idle) {
        if (hasNbrInState({State::Root, State::Follower})) {
            state = State::Follower;
            followDir = labelOfFirstNbrInState({State::Root, State::Follower});
            return true;
        }
    }

    return false;
}

bool ShapeFormationFaultTolerantParticle::tryFollowerRecoveryByPropagation() {
    int maxLabels = isContracted() ? 6 : 10;

    if (safeState == SafeState::Safe) {
        // Try to reconnect to a valid parent candidate
        for (int label = 0; label < maxLabels; label++) {
            if (hasNbrAtLabel(label)) {
                ShapeFormationFaultTolerantParticle& nbr = nbrAtLabel(label);
                if (nbr.state == State::Root || (nbr.state == State::Follower && (nbr.pathToRootState == PathToRootState::Valid) && hasInvalidated[label])) {
                    state = State::Follower;
                    makeHeadLabel(label);
                    followDir = labelToDir(label);
                    return true;
                }
            }
        }

        // Check whether new parent candidates have emerged in the neighborhood in the meantime
        for (int label = 0; label < maxLabels; label++) {
            if (!hasInvalidated[label] && hasNbrAtLabel(label)) {
                ShapeFormationFaultTolerantParticle& nbr = nbrAtLabel(label);
                if (nbr.state == State::Follower && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir))) {
                    nbr.pathToRootState = PathToRootState::Invalidate;
                    hasInvalidated[label] = true;
                }
            }
        }
    }

    return false;
}

void ShapeFormationFaultTolerantParticle::propagateInvalidate() {
    ShapeFormationFaultTolerantParticle& parent = nbrAtLabel(dirToHeadLabel(followDir));
    if (parent.state == State::Follower || parent.state == State::Error) {
        parent.pathToRootState = PathToRootState::Invalidate;
    }
}

void ShapeFormationFaultTolerantParticle::propagateValid() {
    for (const int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if ((nbr.pathToRootState == PathToRootState::Invalid) && (nbr.state == State::Follower) && pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir))) {
                nbr.pathToRootState = PathToRootState::Valid;
            }
        }
    }
}

void ShapeFormationFaultTolerantParticle::updateFlags() {
    // Compute safe state for border:

    safeFlags = {};
    SafeState currentSafeState = SafeState::Safe;

    if (hasNbrInState({State::Retired, State::Seed})) { // particle has a finished neighbour
        currentSafeState = SafeState::Unsafe;
    }
    // Remark: Since currentSafeState is initialized with SafeState::Safe, the following two branches are redundant:
    else if (hasNbrInState({State::Idle, State::Follower, State::Root})) {
        currentSafeState = SafeState::Safe;
    } else {
        for (const int label : uniqueLabels()) {
            if (!hasNbrAtLabel(label)) { // particle has an empty neighboring node
                currentSafeState = SafeState::Safe;
                break;
            }
        }
    }

    // Compute safe state for two rounds:

    for (int round = 0; round < 2; round++) {
        SafeState previousSafeState = currentSafeState;

        for (int label = 0; label < (isContracted() ? 6 : 10); label++) {
            int dir = labelToDir(label);
            int index = 0;
            if (isExpanded() && isTailLabel(label)) {
                index = 1;
            }
            if (hasNbrAtLabel(label)) {
                auto nbr = nbrAtLabel(label);
                if (nbr.state == State::Error) {
                    // Update flag for error neighbour:
                    int nbrIndex = 0;
                    if (nbr.isExpanded() && nbr.pointsAtMyTail(*this, label)) {
                        nbrIndex = 1;
                    }

                    SafeState flag = nbr.safeFlags[dirToNbrDir(nbr, dir)][round][nbrIndex];
                    safeFlags[dir][round][index] = std::min(previousSafeState, flag);
                } else { // Update flag for non-error neighbour
                    safeFlags[dir][round][index] = previousSafeState;
                }
            } else { // Update flag for empty neighboring node
                safeFlags[dir][round][index] = previousSafeState;
            }

            currentSafeState = std::min(currentSafeState, safeFlags[dir][round][index]);
        }

        // in case of an expanded particle: work as if there were two individual particles (head and tail) which have to exchange their flags:
        if (isExpanded()) {
            // read tail flag + write head flag:
            SafeState flag = safeFlags[tailDir()][round][1];
            safeFlags[tailDir()][round][0] = std::min(previousSafeState, flag);
            // read head flag + write tail flag:
            int headDir = (tailDir() + 3) % 6;
            flag = safeFlags[headDir][round][0];
            safeFlags[headDir][round][1] = std::min(previousSafeState, flag);
        }
    }

    // Update safeState:

    safeState = currentSafeState;
}

int ShapeFormationFaultTolerantParticle::particleColor() const {
    // color pool:
    // raspberry: 0xff0077, turquoise:0x00ff77, bright magenta/fuchsia:0xff00ff, spring green:0x77ff00, yellow: 0xffff00, cyan: 0x00ffff, orange: 0xff7700

    if (state == State::Crashed) {
        return 0xff00ff; // bright magenta/fuchsia
    }

    if (state == State::Error) {
        switch (safeState) {
        case SafeState::Unsafe: return 0xff00ff; // bright magenta/fuchsia
        case SafeState::Safe: return 0x00ff00; // bright green/lime
        case SafeState::Undetermined: return 0xffffff; // white
        }
    }

    switch (pathToRootState) {
    case PathToRootState::Invalid: return 0xff7700; // orange
    case PathToRootState::Invalidate: return 0xffff00; // yellow
    case PathToRootState::Valid: return 0x000000; // black
    }

    return -1;
}

int ShapeFormationFaultTolerantParticle::particleCenterColor() const {
    if (state == State::Error && pathToRootState == PathToRootState::Invalidate) {
        return 0xffff00;
    }

    return -1;
}

int ShapeFormationFaultTolerantParticle::headMarkColor() const {
    switch(state) {
    case State::Seed: return 0x00ff00; // bright green/lime
    case State::Idle: return -1;
    case State::Follower: return 0x0000ff; // blue
    case State::Root: return 0xff0000; // red
    case State::Retired: return 0x000000; // black
    case State::Crashed: return 0xff00ff; // bright magenta/fuchsia
    case State::Error: return 0xff00ff; // bright magenta/fuchsia
    }

    return -1;
}

int ShapeFormationFaultTolerantParticle::headMarkDir() const {
    if (isFinished()) {
        return constructionDir;
    } else if (state == State::Root) {
        return moveDir;
    } else if (state == State::Follower) {
        return followDir;
    }

    return -1;
}

int ShapeFormationFaultTolerantParticle::tailMarkColor() const {
    return headMarkColor();
}

std::array<std::array<std::array<VisFlagStates, 2>, 2>, 6> ShapeFormationFaultTolerantParticle::getSafeFlags() const {
  std::array<std::array<std::array<VisFlagStates, 2>, 2>, 6> flags = {};

  if (state == State::Error) {
      for (int localDir = 0; localDir < 6; localDir++) {
          int globalDir = localToGlobalDir(localDir);
          for (int round = 0; round < 2; round++) {
              for (int index = 0; index < (isContracted() ? 1 : 2); index++) {
                  // don't draw flag marks on the connecting line between the two nodes of an expanded particle
                  if (isExpanded() && ((index == 0 && head.nodeInDir(globalDir) == tail()) || (index == 1 && tail().nodeInDir(globalDir) == head))) {
                      continue;
                  }

                  VisFlagStates visFlagState;

                  switch (safeFlags[localDir][round][index]) {
                  case SafeState::Undetermined: visFlagState = VisFlagStates::SafeState_Undetermined; break;
                  case SafeState::Unsafe: visFlagState = VisFlagStates::SafeState_Unsafe; break;
                  case SafeState::Safe: visFlagState = VisFlagStates::SafeState_Safe; break;
                  }

                  if (safeState == SafeState::Safe && round == 1) {
                      int label = 0;
                      if (index == 0) {
                          label = dirToHeadLabel(localDir);
                      } else if (index == 1) {
                          label = dirToTailLabel(localDir);
                      }

                      visFlagState = hasInvalidated[label] ? VisFlagStates::HasInvalidated_True : visFlagState;
                  }

                  flags[globalDir][round][index] = visFlagState;
              }
          }
      }
  }

  return flags;
}

std::array<int, 18> ShapeFormationFaultTolerantParticle::borderColors() const {
    return _borderColors;
}

void ShapeFormationFaultTolerantParticle::updateBorderColors() {
    if (state == State::Retired && !_borderColorsSet) {
        _borderColorsSet = true;

        int closedColor = 0xb8b8b8; // silver
        int openColor = 0xf0f0f0; // light gray

        int receiveDir = constructionReceiveDir();
        int seedConstructionDir = globalToLocalDir(system.seedOrientation());

        if (receiveDir != - 1) {
            int predConstructionDir = (receiveDir + 3) % 6;

            if (nbrAtLabel(receiveDir).state == State::Seed) {
                _borderColors.at((3 * localToGlobalDir((constructionDir + 2) % 6) + 17) % 18) = closedColor;
            }

            if (predConstructionDir == seedConstructionDir && constructionDir != seedConstructionDir) {
                _borderColors.at(3 * localToGlobalDir((seedConstructionDir + 0) % 6) + 2) = closedColor;
                _borderColors.at(3 * localToGlobalDir((predConstructionDir + 3) % 6) + 0) = openColor;
                _borderColors.at(3 * localToGlobalDir((predConstructionDir + 1) % 6) + 1) = openColor;
            } else if (constructionDir == predConstructionDir) {
                _borderColors.at(3 * localToGlobalDir((predConstructionDir + 1) % 6) + 2) = closedColor;
                _borderColors.at(3 * localToGlobalDir((constructionDir + 2) % 6) + 1) = closedColor;
            } else {
                _borderColors.at(3 * localToGlobalDir((predConstructionDir + 1) % 6) + 2) = closedColor;
                _borderColors.at(3 * localToGlobalDir((predConstructionDir + 2) % 6) + 1) = closedColor;
                _borderColors.at((3 * localToGlobalDir((constructionDir + 2) % 6) + 17) % 18) = closedColor;
            }
        }
    }
}

QString ShapeFormationFaultTolerantParticle::inspectionText() const {
    QString text;
    text += "head: (" + QString::number(head.x) + ", " + QString::number(head.y) +
            ")\n";
    text += "orientation: " + QString::number(orientation) + "\n";
    text += "globalTailDir: " + QString::number(globalTailDir) + "\n";
    text += "state: ";
    text += [this](){
        switch(state) {
        case State::Seed:   return "Seed";
        case State::Idle:   return "Idle";
        case State::Follower: return "Follower";
        case State::Root:   return "Root";
        case State::Retired: return "Retired";
        case State::Crashed:  return "Crashed";
        case State::Error: return "Error";
        default:            return "no state";
        }
    }();
    text += "\n";
    text += "pathToRootState: ";
    text += [this](){
        switch(pathToRootState) {
        case PathToRootState::Invalidate:           return "Invalidate";
        case PathToRootState::Invalid:       return "Invalid";
        case PathToRootState::Valid:  return "Valid";
        default:                                return "no path to root state";
        }
    }();
    text += "\n";
    text += "safeState: ";
    text += [this](){
        switch(safeState) {
        case SafeState::Undetermined:   return "Undetermined";
        case SafeState::Unsafe:         return "Unsafe";
        case SafeState::Safe:           return "Safe";
        default:                        return "no safe state";
        }
    }();
    text += "\n";
    text += "constructionDir: " + QString::number(constructionDir) + "\n";
    text += "moveDir: " + QString::number(moveDir) + "\n";
    text += "followDir: " + QString::number(followDir) + "\n";
    text += "turnSignal: " + QString::number(turnSignal) + "\n";
    text += "shape: ";
    text += [this](){
        if (mode == "h") {
            return "hexagon";
        } else if (mode == "s") {
            return "square";
        } else if (mode == "t1") {
            return "vertex triangle";
        } else if (mode == "t2") {
            return "center triangle";
        } else if (mode == "l") {
            return "line";
        } else {
            return "ERROR";
        }
    }();
    text += "\n";

    return text;
}

ShapeFormationFaultTolerantParticle& ShapeFormationFaultTolerantParticle::nbrAtLabel(int label) const {
    return AmoebotParticle::nbrAtLabel<ShapeFormationFaultTolerantParticle>(label);
}

int ShapeFormationFaultTolerantParticle::labelOfFirstNbrInState(
        std::initializer_list<State> states, int startLabel, bool ignoreErrorParticles) const {
    auto prop = [&](const ShapeFormationFaultTolerantParticle& p) {
        for (auto state : states) {
            if (p.state == state) {
                return true;
            }
        }
        return false;
    };

    return labelOfFirstNbrWithProperty<ShapeFormationFaultTolerantParticle>(prop, startLabel, ignoreErrorParticles);
}

bool ShapeFormationFaultTolerantParticle::hasNbrInState(std::initializer_list<State> states)
const {
    return labelOfFirstNbrInState(states) != -1;
}

bool ShapeFormationFaultTolerantParticle::isInState(std::initializer_list<State> states) const {
    for (auto _state : states) {
        if (_state == state) {
            return true;
        }
    }

    return false;
}

int ShapeFormationFaultTolerantParticle::constructionReceiveDir() const {
    auto prop = [&](const ShapeFormationFaultTolerantParticle& p) {
        if (p.mode == "l") {
            return isContracted() &&
                    p.isFinished() &&
                    (pointsAtMe(p, p.constructionDir) ||
                     pointsAtMe(p, (p.constructionDir + 3) % 6));
        } else {
            return isContracted() &&
                    p.isFinished() &&
                    pointsAtMe(p, p.constructionDir);
        }
    };

    return labelOfFirstNbrWithProperty<ShapeFormationFaultTolerantParticle>(prop);
}

bool ShapeFormationFaultTolerantParticle::canRetire() const {
    return constructionReceiveDir() != -1;
}

bool ShapeFormationFaultTolerantParticle::isFinished() const {
    return isInState({State::Retired, State::Seed});
}

// Currently not used by the fault tolerant (hexagon) shape formation algorithm
bool ShapeFormationFaultTolerantParticle::updateConstructionDir() {
    int newConstructionDir;
    int newTurnSignal = -1;

    if (mode == "h") {  // Hexagon construction.
        newConstructionDir = constructionReceiveDir();
        if (nbrAtLabel(newConstructionDir).state == State::Seed) {
            newConstructionDir = (newConstructionDir + 1) % 6;
        } else {
            newConstructionDir = (newConstructionDir + 2) % 6;
        }

        if (hasNbrAtLabel(newConstructionDir)) {
            auto nbr = nbrAtLabel(newConstructionDir);
            if (nbr.isErrorParticle()) {
                return false;
            } else if (nbr.state == State::Retired) {
                newConstructionDir = (newConstructionDir + 1) % 6;
            }
        }
    } else if (mode == "s") {  // Square construction.
        newConstructionDir = constructionReceiveDir();
        if (nbrAtLabel(newConstructionDir).state == State::Seed) {
            newConstructionDir = (newConstructionDir + 1) % 6;
        } else if (nbrAtLabel(newConstructionDir).turnSignal == 0) {
            newConstructionDir = (newConstructionDir + 2) % 6;
            newTurnSignal = 1;
        } else if (nbrAtLabel(newConstructionDir).turnSignal == 1) {
            newConstructionDir = (newConstructionDir + 1) % 6;
            newTurnSignal = 0;
        }

        if (hasNbrAtLabel(newConstructionDir)) {
            auto nbr = nbrAtLabel(newConstructionDir);
            if (nbr.isErrorParticle()) {
                return false;
            } else if (nbr.isFinished()) {
                if (newTurnSignal == 1) {
                    newTurnSignal = 0;
                    newConstructionDir = (newConstructionDir + 1) % 6;
                } else if (newTurnSignal == 0) {
                    newTurnSignal = 1;
                    newConstructionDir = (newConstructionDir + 2) % 6;
                }
            }
        }
    } else if (mode == "t1") {  // Vertex Triangle construction.
        newConstructionDir = constructionReceiveDir();

        if (nbrAtLabel(newConstructionDir).state == State::Seed) {
            newConstructionDir = (newConstructionDir + 5) % 6;
        } else {
            int labelOfFirstNbr = labelOfFirstNbrInState({State::Retired, State::Seed}, (newConstructionDir + 5) % 6, false);

            if (labelOfFirstNbr == -1) {
                return false;
            }

            int labelOfSecondNbr = -1;

            if (labelOfFirstNbr == (newConstructionDir + 5) % 6) {
                labelOfSecondNbr = labelOfFirstNbrInState({State::Retired}, (newConstructionDir + 4) % 6, false);
            } else {
                labelOfFirstNbr = labelOfFirstNbrInState({State::Retired}, (newConstructionDir + 1) % 6, false);
                labelOfSecondNbr = labelOfFirstNbrInState({State::Retired}, (newConstructionDir + 2) % 6, false);
            }

            if (labelOfFirstNbr == -1 || labelOfSecondNbr == -1) {
                return false;
            }

            if ((labelOfFirstNbr == (newConstructionDir + 5) % 6 &&
                        labelOfSecondNbr == (newConstructionDir + 4) % 6) ||
                       (labelOfFirstNbr == (newConstructionDir + 1) % 6 &&
                        labelOfSecondNbr == (newConstructionDir + 2) % 6)) {
                newConstructionDir = (newConstructionDir + 3) % 6;
            } else if (labelOfFirstNbr == (newConstructionDir + 5) % 6) {
                newConstructionDir = (newConstructionDir + 2) % 6;
                newTurnSignal = 1;
            } else if (labelOfFirstNbr == (newConstructionDir + 1) % 6) {
                newConstructionDir = (newConstructionDir + 4) % 6;
                newTurnSignal = 0;
            } else if (nbrAtLabel(newConstructionDir).turnSignal == 0) {
                newConstructionDir = (newConstructionDir + 5) % 6;
            } else if (nbrAtLabel(newConstructionDir).turnSignal == 1) {
                newConstructionDir = (newConstructionDir + 1) % 6;
            }
        }
    } else if (mode == "t2") {  // Center Triangle construction.
        newConstructionDir = (constructionReceiveDir() + 1) % 6;

        if (hasNbrAtLabel(newConstructionDir)) {
            auto nbr = nbrAtLabel(newConstructionDir);
            if (nbr.isErrorParticle()) {
                return false;
            } else if (nbr.isFinished()) {
                newConstructionDir = (newConstructionDir + 2) % 6;
            }
        }
    } else if (mode == "l") {  // Line construction.
        newConstructionDir = (constructionReceiveDir() + 3) % 6;
    } else {
        // This is executing in an invalid mode.
        Q_ASSERT(false);
    }

    constructionDir = newConstructionDir;
    if (newTurnSignal != -1) {
        turnSignal = newTurnSignal;
    }

    return true;
}

int ShapeFormationFaultTolerantParticle::computeMoveLabel() const {
    int startLabel = labelOfFirstNbrInState({State::Seed, State::Retired}); // compute label of a finished neighbour, if there is one.
    if (startLabel == -1) {
        return -1;
    }

    const int labelLimit = isContracted() ? 6 : 10;
    int label = startLabel;

    while (hasNbrAtLabel(label) && (nbrAtLabel(label).isFinished())) {
        label = (labelLimit + label - 1) % labelLimit;
        if (label == startLabel) {
            return -1;
        }
    }

    if (hasNbrAtLabel(label) && (nbrAtLabel(label).isErrorParticle())) {
        return -1;
    }

    return label;
}

bool ShapeFormationFaultTolerantParticle::updateMoveDir() {
    int label = computeMoveLabel();
    if (label != -1) {
        makeHeadLabel(label);
        moveDir = labelToDir(label);

        return true;
    }

    return false;
}

bool ShapeFormationFaultTolerantParticle::hasTailFollower() const {
    auto prop = [&](const ShapeFormationFaultTolerantParticle& p) {
        return p.state == State::Follower &&
                pointsAtMyTail(p, p.dirToHeadLabel(p.followDir));
    };

    return labelOfFirstNbrWithProperty<ShapeFormationFaultTolerantParticle>(prop) != -1;
}

bool ShapeFormationFaultTolerantParticle::hasBlockingTailNbr() const {
    if (hasTailFollower()) {
        return true;
    }

    for (int label : tailExclusiveLabels()) {
        if (hasNbrAtLabel(label)) {
            auto nbr = nbrAtLabel(label);
            if (nbr.state == State::Idle || nbr.isErrorParticle()) {
                return true;
            }
        }
    }

    return false;
}

ShapeFormationFaultTolerantSystem::ShapeFormationFaultTolerantSystem(int numParticles, double holeProb,
                                                                     QString mode) {
    Q_ASSERT(mode == "h" || mode == "s" || mode == "t1" || mode == "t2" ||
             mode == "l");
    Q_ASSERT(numParticles > 0);
    Q_ASSERT(0 <= holeProb && holeProb <= 1);

    // Insert the seed at (0,0).
    _seedOrientation = randDir();
    insert(new ShapeFormationFaultTolerantParticle(Node(0, 0), -1, seedOrientation(), *this,
                                                   ShapeFormationFaultTolerantParticle::State::Seed, mode));
    std::set<Node> occupied;
    occupied.insert(Node(0, 0));

    std::set<Node> candidates;
    for (int i = 0; i < 6; ++i) {
        candidates.insert(Node(0, 0).nodeInDir(i));
    }

    // Add inactive particles.
    int numNonStaticParticles = 0;
    while (numNonStaticParticles < numParticles && !candidates.empty()) {
        // Pick random candidate.
        int randIndex = randInt(0, candidates.size());
        Node randomCandidate;
        for (auto it = candidates.begin(); it != candidates.end(); ++it) {
            if (randIndex == 0) {
                randomCandidate = *it;
                candidates.erase(it);
                break;
            } else {
                randIndex--;
            }
        }

        occupied.insert(randomCandidate);

        // Add this candidate as a particle if not a hole.
        if (randBool(1.0 - holeProb)) {
            insert(new ShapeFormationFaultTolerantParticle(randomCandidate, -1, randDir(), *this,
                                                           ShapeFormationFaultTolerantParticle::State::Idle,
                                                           mode));

            ++numNonStaticParticles;

            // Add new candidates.
            for (int i = 0; i < 6; ++i) {
                auto neighbor = randomCandidate.nodeInDir(i);
                if (occupied.find(neighbor) == occupied.end()) {
                    candidates.insert(neighbor);
                }
            }
        }
    }
}

bool ShapeFormationFaultTolerantSystem::hasTerminated() const {
#ifdef QT_DEBUG
    if (!isConnected(particles)) {
        return true;
    }
#endif

    for (auto p : particles) {
        auto hp = dynamic_cast<ShapeFormationFaultTolerantParticle*>(p);
        if (!hp->isFinished() || (hp->pathToRootState != ShapeFormationFaultTolerantParticle::PathToRootState::Valid)) {
            return false;
        }
    }

    return true;
}

void ShapeFormationFaultTolerantSystem::nuke() {
    for (auto p : particles) {
        auto hp = dynamic_cast<ShapeFormationFaultTolerantParticle*>(p);
        if (hp->state != ShapeFormationFaultTolerantParticle::State::Seed) {
            crashParticleAt(hp->head);
        }
    }
}

std::set<QString> ShapeFormationFaultTolerantSystem::getAcceptedModes() {
    std::set<QString> set = {"h", "t1", "t2", "s", "l"};
    return set;
}

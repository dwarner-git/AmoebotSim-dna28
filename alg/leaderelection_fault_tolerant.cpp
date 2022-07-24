/* Copyright (C) 2021 Daniel Warner.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "alg/leaderelection_fault_tolerant.h"

#include <QVariant>
#include <QtGlobal>

LeaderElectionFaultTolerantParticle::LeaderElectionFaultTolerantParticle(const Node head,
                                                                         const int globalTailDir,
                                                                         const int orientation,
                                                                         AmoebotSystem& system,
                                                                         State state, const QString mode)
    : AmoebotParticle(head, globalTailDir, orientation, system),

      electionState(ElectionState::Candidate),
      electionPhase(ElectionPhase::StartAndInitializeSolitudeVerification),
      showLeaderElection(true),
      priority(0),
      parentLabel(-1),
      mergeParticleLabel(-1),
      mergeParticlePathLabel(-1),
      treeColor(-1),
      rootOffset(0),
      edgeFlags({}),
      edgeBalance({}),
      balance({}),

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
        electionState = ElectionState::Candidate;
        electionPhase = ElectionPhase::Start;
        priority = 2;

        constructionDir = 0;
    }
}

bool LeaderElectionFaultTolerantParticle::isErrorParticle() const {
    return isInState({State::Crashed, State::Error});
}

bool LeaderElectionFaultTolerantParticle::hasErrorNbr() const {
    auto prop = [&](const LeaderElectionFaultTolerantParticle& p) {
        return p.isErrorParticle();
    };

    return labelOfFirstNbrWithProperty<LeaderElectionFaultTolerantParticle>(prop) != -1;
}

void LeaderElectionFaultTolerantParticle::crash() {
    state = State::Crashed;
}

void LeaderElectionFaultTolerantParticle::updateBalance(int &x, int &y, int dir) {
    switch (dir) {
        case 0: x += 1; break;
        case 1: y += 1; break;
        case 2: x -= 1; y += 1; break;
        case 3: x -= 1; break;
        case 4: y -= 1; break;
        case 5: x += 1; y -= 1; break;
    }
}

void LeaderElectionFaultTolerantParticle::performElection() {
}

void LeaderElectionFaultTolerantParticle::activate() {
}

void LeaderElectionFaultTolerantParticle::performMovement() {
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

bool LeaderElectionFaultTolerantParticle::tryToBecomeRetired_Hexagon() {
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
                    LeaderElectionFaultTolerantParticle& nbr = nbrAtLabel(innerDir);
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
                LeaderElectionFaultTolerantParticle& nbr = nbrAtLabel(seedLabel);
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

bool LeaderElectionFaultTolerantParticle::tryToBecomeRoot_Hexagon() {
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
            LeaderElectionFaultTolerantParticle& nbrS = nbrAtLabel(labelNbrS);
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
                auto prop = [&](const LeaderElectionFaultTolerantParticle& p) {
                    return p.isFinished() && pointsAtMe(p, p.constructionDir);
                };

                int label = labelOfFirstNbrWithProperty<LeaderElectionFaultTolerantParticle>(prop, 0);
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

bool LeaderElectionFaultTolerantParticle::tryToBecomeFollower() {
    if (state == State::Idle) {
        if (hasNbrInState({State::Root, State::Follower})) {
            state = State::Follower;
            followDir = labelOfFirstNbrInState({State::Root, State::Follower});
            return true;
        }
    }

    return false;
}

bool LeaderElectionFaultTolerantParticle::tryFollowerRecoveryByPropagation() {
    int maxLabels = isContracted() ? 6 : 10;

    if (safeState == SafeState::Safe) {
        // Try to reconnect to a valid parent candidate
        for (int label = 0; label < maxLabels; label++) {
            if (hasNbrAtLabel(label)) {
                LeaderElectionFaultTolerantParticle& nbr = nbrAtLabel(label);
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
                LeaderElectionFaultTolerantParticle& nbr = nbrAtLabel(label);
                if (nbr.state == State::Follower && !pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir))) {
                    nbr.pathToRootState = PathToRootState::Invalidate;
                    hasInvalidated[label] = true;
                }
            }
        }
    }

    return false;
}

void LeaderElectionFaultTolerantParticle::propagateInvalidate() {
    LeaderElectionFaultTolerantParticle& parent = nbrAtLabel(dirToHeadLabel(followDir));
    if (parent.state == State::Follower || parent.state == State::Error) {
        parent.pathToRootState = PathToRootState::Invalidate;
    }
}

void LeaderElectionFaultTolerantParticle::propagateValid() {
    for (const int label : uniqueLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if ((nbr.pathToRootState == PathToRootState::Invalid) && (nbr.state == State::Follower) && pointsAtMe(nbr, nbr.dirToHeadLabel(nbr.followDir))) {
                nbr.pathToRootState = PathToRootState::Valid;
            }
        }
    }
}

void LeaderElectionFaultTolerantParticle::updateFlags() {
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

int LeaderElectionFaultTolerantParticle::particleColor() const {
    if (electionState == ElectionState::Candidate) {
        return 0x00ff00;
    }

    if (electionState == ElectionState::Child) {
        switch (balance[0]) {
            case -2: return 0xff0000; // red
            case -1: return 0xff7619; // orange
            case 0: return 0; // black
            case 1: return 0x77ff00; // spring green
            case 2: return 0x00ff00; // bright green/lime
        }
    }

    return treeColor;

    if (showLeaderElection) {
        if (state == State::Crashed) {
            return 0xff00ff; // bright magenta/fuchsia
        }

        if (electionState == ElectionState::Candidate) {
            switch (priority) {
            case 0: return 0xff6700; // orange
            case 1: return 0x00ff00; // bright green/lime
            case 2: return 0xff0000; // red
            }

            return 0x00ff00;
        }

        if (electionState == ElectionState::Child) {
            switch (priority) {
            case 0: return 0xff7619; // orange
            case 1: return 0x77ff00; // spring green
            case 2: return 0xff0000; // red
            }
        }

        return 0x000000;
    } else {
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
    }

    return -1;
}

int LeaderElectionFaultTolerantParticle::particleCenterColor() const {
    if (showLeaderElection) {
        if (priority == 0) {
            return 0xffff00;
        }

    } else {
        if (state == State::Error && pathToRootState == PathToRootState::Invalidate) {
            return 0xffff00;
        }
    }

    return -1;
}

int LeaderElectionFaultTolerantParticle::headMarkColor() const {

    if (showLeaderElection) {
        if (electionState == ElectionState::Candidate) {
            return 0x00ff00;
        }

        return treeColor;

        return 0;

            if (state != State::Idle) {
                return treeColor;
            } else {
                return -1;
            }
    } else {
        switch(state) {
        case State::Seed: return 0x00ff00; // bright green/lime
        case State::Idle: return -1;
        case State::Follower: return 0x0000ff; // blue
        case State::Root: return 0xff0000; // red
        case State::Retired: return 0x000000; // black
        case State::Crashed: return 0xff00ff; // bright magenta/fuchsia
        case State::Error: return 0xff00ff; // bright magenta/fuchsia
        }
    }

    return -1;
}

int LeaderElectionFaultTolerantParticle::headMarkDir() const {
    if (showLeaderElection) {
        if (electionState == ElectionState::Child) {
            return parentLabel;
        }
    } else {
        if (isFinished()) {
            return constructionDir;
        } else if (state == State::Root) {
            return moveDir;
        } else if (state == State::Follower) {
            return followDir;
        }
    }

    return -1;
}

int LeaderElectionFaultTolerantParticle::tailMarkColor() const {
    return headMarkColor();
}

std::array<std::array<std::array<VisFlagStates, 2>, 2>, 6> LeaderElectionFaultTolerantParticle::getSafeFlags() const {
  std::array<std::array<std::array<VisFlagStates, 2>, 2>, 6> flags = {};

  for (int localDir = 0; localDir < 6; localDir++) {
      int globalDir = localToGlobalDir(localDir);
      for (int round = 0; round < 1; round++) {
          for (int index = 0; index < (isContracted() ? 1 : 2); index++) {
              // don't draw flag marks on the connecting line between the two nodes of an expanded particle
              if (isExpanded() && ((index == 0 && head.nodeInDir(globalDir) == tail()) || (index == 1 && tail().nodeInDir(globalDir) == head))) {
                  continue;
              }

              int label = 0;
              if (index == 0) {
                  label = dirToHeadLabel(localDir);
              } else if (index == 1) {
                  label = dirToTailLabel(localDir);
              }

              VisFlagStates visFlagState = VisFlagStates::SafeState_Undetermined;

              switch (edgeBalance[label][round]) {
                  case -2: visFlagState = VisFlagStates::SafeState_Unsafe; break;
                  case -1: visFlagState = VisFlagStates::SafeState_Unsafe; break;
                  case 0: visFlagState = VisFlagStates::SafeState_Undetermined; break;
                  case 1: visFlagState = VisFlagStates::SafeState_Safe; break;
                  case 2: visFlagState = VisFlagStates::SafeState_Safe; break;
              }

              flags[globalDir][round][index] = visFlagState;
          }
      }
  }

  return flags;
}

std::array<int, 18> LeaderElectionFaultTolerantParticle::borderColors() const {
    return _borderColors;
}

void LeaderElectionFaultTolerantParticle::updateBorderColors() {
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

QString LeaderElectionFaultTolerantParticle::inspectionText() const {
    QString text;
    text += "head: (" + QString::number(head.x) + ", " + QString::number(head.y) +
            ")\n";
    text += "orientation: " + QString::number(orientation) + "\n";
    text += "globalTailDir: " + QString::number(globalTailDir) + "\n";

    text += "electionState: ";
    text += [this](){
        switch(electionState) {
        case ElectionState::Candidate:      return "Candidate";
        case ElectionState::Child:          return "Child";
        case ElectionState::Crashed:        return "Crashed";
        default:                            return "no electionState";
        }
    }();
    text += "\n";
    text += "electionPhase: ";
    text += [this](){
        switch(electionPhase) {
            case ElectionPhase::Start:                     return "Start";
            case ElectionPhase::StartAndInitializeSolitudeVerification:     return "StartAndInitializeSolitudeVerification";
            case ElectionPhase::PropagatePriority:         return "PropagatePriority";
            case ElectionPhase::PropagatePriorityAndInitializeSolitudeVerification:         return "PropagatePriorityAndInitializeSolitudeVerification";
            case ElectionPhase::EnablePriority:            return "EnablePriority";
            case ElectionPhase::SearchForMergeParticle:    return "SearchForMergeParticle";
            case ElectionPhase::SolitudeVerificationSearchForSource:      return "SolitudeVerificationSearchForSource";
            case ElectionPhase::SolitudeVerificationFixSource:            return "SolitudeVerificationFixSource";
            case ElectionPhase::SolitudeVerificationSearchForTarget:      return "SolitudeVerificationSearchForTarget";
            case ElectionPhase::SolitudeVerificationFixTarget:            return "SolitudeVerificationFixTarget";
            case ElectionPhase::MoveRootToMergeParticle:   return "MoveRootToMergeParticle";
            case ElectionPhase::TreeMergeOccured:          return "TreeMergeOccured";
            case ElectionPhase::ChildCrashed:              return "ChildCrashed";
            default:                                       return "no electionPhase";
        }
    }();
    text += "\n";
    text += "treeState: ";
    text += [this](){
        switch(treeState) {
        case TreeState::Ack:                     return "Ack";
        case TreeState::PropagateDown:           return "PropagateDown";
        case TreeState::PropagateUp:             return "PropagateUp";
        case TreeState::WaitingForAck:           return "WaitingForAck";
        default:                                 return "no treeState";
        }
    }();
    text += "\n";
    text += "priorityEnabled: " + QVariant(priorityEnabled).toString() + "\n";
    text += "priority: " + QString::number(priority) + "\n";
    text += "isDominatingTree: " + QVariant(isDominatingTree).toString() + "\n";
    text += "parentLabel: " + QString::number(parentLabel) + "\n";
    text += "mergeParticleLabel: " + QString::number(mergeParticleLabel) + "\n";
    text += "mergeParticlePathLabel: " + QString::number(mergeParticlePathLabel) + "\n";
    text += "sourceParticleLabel: " + QString::number(sourceParticleLabel) + "\n";
    text += "sourceParticlePathLabel: " + QString::number(sourceParticlePathLabel) + "\n";
    text += "targetParticleLabel: " + QString::number(targetParticleLabel) + "\n";
    text += "targetParticlePathLabel: " + QString::number(targetParticlePathLabel) + "\n";
    text += "treeColor: " + QString::number(treeColor) + "\n";
    text += "rootOffset: " + QString::number(rootOffset) + "\n";

    text += "state: ";
    text += [this](){
        switch(state) {
        case State::Seed:     return "Seed";
        case State::Idle:     return "Idle";
        case State::Follower: return "Follower";
        case State::Root:     return "Root";
        case State::Retired:  return "Retired";
        case State::Crashed:  return "Crashed";
        case State::Error:    return "Error";
        default:              return "no state";
        }
    }();
    text += "\n";
    text += "pathToRootState: ";
    text += [this](){
        switch(pathToRootState) {
        case PathToRootState::Invalidate:           return "Invalidate";
        case PathToRootState::Invalid:              return "Invalid";
        case PathToRootState::Valid:                return "Valid";
        default:                                    return "no path to root state";
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

LeaderElectionFaultTolerantParticle& LeaderElectionFaultTolerantParticle::nbrAtLabel(int label) const {
    return AmoebotParticle::nbrAtLabel<LeaderElectionFaultTolerantParticle>(label);
}

int LeaderElectionFaultTolerantParticle::labelOfFirstNbrInState(
        std::initializer_list<State> states, int startLabel, bool ignoreErrorParticles) const {
    auto prop = [&](const LeaderElectionFaultTolerantParticle& p) {
        for (auto state : states) {
            if (p.state == state) {
                return true;
            }
        }
        return false;
    };

    return labelOfFirstNbrWithProperty<LeaderElectionFaultTolerantParticle>(prop, startLabel, ignoreErrorParticles);
}

bool LeaderElectionFaultTolerantParticle::hasNbrInState(std::initializer_list<State> states)
const {
    return labelOfFirstNbrInState(states) != -1;
}

bool LeaderElectionFaultTolerantParticle::isInState(std::initializer_list<State> states) const {
    for (auto _state : states) {
        if (_state == state) {
            return true;
        }
    }

    return false;
}

int LeaderElectionFaultTolerantParticle::constructionReceiveDir() const {
    auto prop = [&](const LeaderElectionFaultTolerantParticle& p) {
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

    return labelOfFirstNbrWithProperty<LeaderElectionFaultTolerantParticle>(prop);
}

bool LeaderElectionFaultTolerantParticle::canRetire() const {
    return constructionReceiveDir() != -1;
}

bool LeaderElectionFaultTolerantParticle::isFinished() const {
    return isInState({State::Retired, State::Seed});
}

int LeaderElectionFaultTolerantParticle::computeMoveLabel() const {
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

bool LeaderElectionFaultTolerantParticle::updateMoveDir() {
    int label = computeMoveLabel();
    if (label != -1) {
        makeHeadLabel(label);
        moveDir = labelToDir(label);

        return true;
    }

    return false;
}

bool LeaderElectionFaultTolerantParticle::hasTailFollower() const {
    auto prop = [&](const LeaderElectionFaultTolerantParticle& p) {
        return p.state == State::Follower &&
                pointsAtMyTail(p, p.dirToHeadLabel(p.followDir));
    };

    return labelOfFirstNbrWithProperty<LeaderElectionFaultTolerantParticle>(prop) != -1;
}

bool LeaderElectionFaultTolerantParticle::hasBlockingTailNbr() const {
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

int LeaderElectionFaultTolerantParticle::distinctColorsIndex = 0;

// stackoverflow.com discussion "Generate distinctly different RGB colors in graphs" (retrieved: 06.03.2021):
// https://stackoverflow.com/questions/309149/generate-distinctly-different-rgb-colors-in-graphs
// Code by user "Tatarize", https://stackoverflow.com/users/631911/tatarize
const std::array<int, 1024> LeaderElectionFaultTolerantParticle::distinctColors =
{
    0x000000, 0xffff00, 0x1ce6ff, 0xff34ff, 0xff4a46, 0x008941, 0x006fa6, 0xa30059,
    0xffdbe5, 0x7a4900, 0x0000a6, 0x63ffac, 0xb79762, 0x004d43, 0x8fb0ff, 0x997d87,
    0x5a0007, 0x809693, 0xfeffe6, 0x1b4400, 0x4fc601, 0x3b5dff, 0x4a3b53, 0xff2f80,
    0x61615a, 0xba0900, 0x6b7900, 0x00c2a0, 0xffaa92, 0xff90c9, 0xb903aa, 0xd16100,
    0xddefff, 0x000035, 0x7b4f4b, 0xa1c299, 0x300018, 0x0aa6d8, 0x013349, 0x00846f,
    0x372101, 0xffb500, 0xc2ffed, 0xa079bf, 0xcc0744, 0xc0b9b2, 0xc2ff99, 0x001e09,
    0x00489c, 0x6f0062, 0x0cbd66, 0xeec3ff, 0x456d75, 0xb77b68, 0x7a87a1, 0x788d66,
    0x885578, 0xfad09f, 0xff8a9a, 0xd157a0, 0xbec459, 0x456648, 0x0086ed, 0x886f4c,
    0x34362d, 0xb4a8bd, 0x00a6aa, 0x452c2c, 0x636375, 0xa3c8c9, 0xff913f, 0x938a81,
    0x575329, 0x00fecf, 0xb05b6f, 0x8cd0ff, 0x3b9700, 0x04f757, 0xc8a1a1, 0x1e6e00,
    0x7900d7, 0xa77500, 0x6367a9, 0xa05837, 0x6b002c, 0x772600, 0xd790ff, 0x9b9700,
    0x549e79, 0xfff69f, 0x201625, 0x72418f, 0xbc23ff, 0x99adc0, 0x3a2465, 0x922329,
    0x5b4534, 0xfde8dc, 0x404e55, 0x0089a3, 0xcb7e98, 0xa4e804, 0x324e72, 0x6a3a4c,
    0x83ab58, 0x001c1e, 0xd1f7ce, 0x004b28, 0xc8d0f6, 0xa3a489, 0x806c66, 0x222800,
    0xbf5650, 0xe83000, 0x66796d, 0xda007c, 0xff1a59, 0x8adbb4, 0x1e0200, 0x5b4e51,
    0xc895c5, 0x320033, 0xff6832, 0x66e1d3, 0xcfcdac, 0xd0ac94, 0x7ed379, 0x012c58,
    0x7a7bff, 0xd68e01, 0x353339, 0x78afa1, 0xfeb2c6, 0x75797c, 0x837393, 0x943a4d,
    0xb5f4ff, 0xd2dcd5, 0x9556bd, 0x6a714a, 0x001325, 0x02525f, 0x0aa3f7, 0xe98176,
    0xdbd5dd, 0x5ebcd1, 0x3d4f44, 0x7e6405, 0x02684e, 0x962b75, 0x8d8546, 0x9695c5,
    0xe773ce, 0xd86a78, 0x3e89be, 0xca834e, 0x518a87, 0x5b113c, 0x55813b, 0xe704c4,
    0x00005f, 0xa97399, 0x4b8160, 0x59738a, 0xff5da7, 0xf7c9bf, 0x643127, 0x513a01,
    0x6b94aa, 0x51a058, 0xa45b02, 0x1d1702, 0xe20027, 0xe7ab63, 0x4c6001, 0x9c6966,
    0x64547b, 0x97979e, 0x006a66, 0x391406, 0xf4d749, 0x0045d2, 0x006c31, 0xddb6d0,
    0x7c6571, 0x9fb2a4, 0x00d891, 0x15a08a, 0xbc65e9, 0xfffffe, 0xc6dc99, 0x203b3c,
    0x671190, 0x6b3a64, 0xf5e1ff, 0xffa0f2, 0xccaa35, 0x374527, 0x8bb400, 0x797868,
    0xc6005a, 0x3b000a, 0xc86240, 0x29607c, 0x402334, 0x7d5a44, 0xccb87c, 0xb88183,
    0xaa5199, 0xb5d6c3, 0xa38469, 0x9f94f0, 0xa74571, 0xb894a6, 0x71bb8c, 0x00b433,
    0x789ec9, 0x6d80ba, 0x953f00, 0x5eff03, 0xe4fffc, 0x1be177, 0xbcb1e5, 0x76912f,
    0x003109, 0x0060cd, 0xd20096, 0x895563, 0x29201d, 0x5b3213, 0xa76f42, 0x89412e,
    0x1a3a2a, 0x494b5a, 0xa88c85, 0xf4abaa, 0xa3f3ab, 0x00c6c8, 0xea8b66, 0x958a9f,
    0xbdc9d2, 0x9fa064, 0xbe4700, 0x658188, 0x83a485, 0x453c23, 0x47675d, 0x3a3f00,
    0x061203, 0xdffb71, 0x868e7e, 0x98d058, 0x6c8f7d, 0xd7bfc2, 0x3c3e6e, 0xd83d66,
    0x2f5d9b, 0x6c5e46, 0xd25b88, 0x5b656c, 0x00b57f, 0x545c46, 0x866097, 0x365d25,
    0x252f99, 0x00ccff, 0x674e60, 0xfc009c, 0x92896b, 0x1e2324, 0xdec9b2, 0x9d4948,
    0x85abb4, 0x342142, 0xd09685, 0xa4acac, 0x00ffff, 0xae9c86, 0x742a33, 0x0e72c5,
    0xafd8ec, 0xc064b9, 0x91028c, 0xfeedbf, 0xffb789, 0x9cb8e4, 0xafffd1, 0x2a364c,
    0x4f4a43, 0x647095, 0x34bbff, 0x807781, 0x920003, 0xb3a5a7, 0x018615, 0xf1ffc8,
    0x976f5c, 0xff3bc1, 0xff5f6b, 0x077d84, 0xf56d93, 0x5771da, 0x4e1e2a, 0x830055,
    0x02d346, 0xbe452d, 0x00905e, 0xbe0028, 0x6e96e3, 0x007699, 0xfec96d, 0x9c6a7d,
    0x3fa1b8, 0x893de3, 0x79b4d6, 0x7fd4d9, 0x6751bb, 0xb28d2d, 0xe27a05, 0xdd9cb8,
    0xaabc7a, 0x980034, 0x561a02, 0x8f7f00, 0x635000, 0xcd7dae, 0x8a5e2d, 0xffb3e1,
    0x6b6466, 0xc6d300, 0x0100e2, 0x88ec69, 0x8fccbe, 0x21001c, 0x511f4d, 0xe3f6e3,
    0xff8eb1, 0x6b4f29, 0xa37f46, 0x6a5950, 0x1f2a1a, 0x04784d, 0x101835, 0xe6e0d0,
    0xff74fe, 0x00a45f, 0x8f5df8, 0x4b0059, 0x412f23, 0xd8939e, 0xdb9d72, 0x604143,
    0xb5bace, 0x989eb7, 0xd2c4db, 0xa587af, 0x77d796, 0x7f8c94, 0xff9b03, 0x555196,
    0x31ddae, 0x74b671, 0x802647, 0x2a373f, 0x014a68, 0x696628, 0x4c7b6d, 0x002c27,
    0x7a4522, 0x3b5859, 0xe5d381, 0xfff3ff, 0x679fa0, 0x261300, 0x2c5742, 0x9131af,
    0xaf5d88, 0xc7706a, 0x61ab1f, 0x8cf2d4, 0xc5d9b8, 0x9ffffb, 0xbf45cc, 0x493941,
    0x863b60, 0xb90076, 0x003177, 0xc582d2, 0xc1b394, 0x602b70, 0x887868, 0xbabfb0,
    0x030012, 0xd1acfe, 0x7fdefe, 0x4b5c71, 0xa3a097, 0xe66d53, 0x637b5d, 0x92bea5,
    0x00f8b3, 0xbeddff, 0x3db5a7, 0xdd3248, 0xb6e4de, 0x427745, 0x598c5a, 0xb94c59,
    0x8181d5, 0x94888b, 0xfed6bd, 0x536d31, 0x6eff92, 0xe4e8ff, 0x20e200, 0xffd0f2,
    0x4c83a1, 0xbd7322, 0x915c4e, 0x8c4787, 0x025117, 0xa2aa45, 0x2d1b21, 0xa9ddb0,
    0xff4f78, 0x528500, 0x009a2e, 0x17fce4, 0x71555a, 0x525d82, 0x00195a, 0x967874,
    0x555558, 0x0b212c, 0x1e202b, 0xefbfc4, 0x6f9755, 0x6f7586, 0x501d1d, 0x372d00,
    0x741d16, 0x5eb393, 0xb5b400, 0xdd4a38, 0x363dff, 0xad6552, 0x6635af, 0x836bba,
    0x98aa7f, 0x464836, 0x322c3e, 0x7cb9ba, 0x5b6965, 0x707d3d, 0x7a001d, 0x6e4636,
    0x443a38, 0xae81ff, 0x489079, 0x897334, 0x009087, 0xda713c, 0x361618, 0xff6f01,
    0x006679, 0x370e77, 0x4b3a83, 0xc9e2e6, 0xc44170, 0xff4526, 0x73be54, 0xc4df72,
    0xadff60, 0x00447d, 0xdccec9, 0xbd9479, 0x656e5b, 0xec5200, 0xff6ec2, 0x7a617e,
    0xddaea2, 0x77837f, 0xa53327, 0x608eff, 0xb599d7, 0xa50149, 0x4e0025, 0xc9b1a9,
    0x03919a, 0x1b2a25, 0xe500f1, 0x982e0b, 0xb67180, 0xe05859, 0x006039, 0x578f9b,
    0x305230, 0xce934c, 0xb3c2be, 0xc0bac0, 0xb506d3, 0x170c10, 0x4c534f, 0x224451,
    0x3e4141, 0x78726d, 0xb6602b, 0x200441, 0xddb588, 0x497200, 0xc5aab6, 0x033c61,
    0x71b2f5, 0xa9e088, 0x4979b0, 0xa2c3df, 0x784149, 0x2d2b17, 0x3e0e2f, 0x57344c,
    0x0091be, 0xe451d1, 0x4b4b6a, 0x5c011a, 0x7c8060, 0xff9491, 0x4c325d, 0x005c8b,
    0xe5fda4, 0x68d1b6, 0x032641, 0x140023, 0x8683a9, 0xcfff00, 0xa72c3e, 0x34475a,
    0xb1bb9a, 0xb4a04f, 0x8d918e, 0xa168a6, 0x813d3a, 0x425218, 0xda8386, 0x776133,
    0x563930, 0x8498ae, 0x90c1d3, 0xb5666b, 0x9b585e, 0x856465, 0xad7c90, 0xe2bc00,
    0xe3aae0, 0xb2c2fe, 0xfd0039, 0x009b75, 0xfff46d, 0xe87eac, 0xdfe3e6, 0x848590,
    0xaa9297, 0x83a193, 0x577977, 0x3e7158, 0xc64289, 0xea0072, 0xc4a8cb, 0x55c899,
    0xe78fcf, 0x004547, 0xf6e2e3, 0x966716, 0x378fdb, 0x435e6a, 0xda0004, 0x1b000f,
    0x5b9c8f, 0x6e2b52, 0x011115, 0xe3e8c4, 0xae3b85, 0xea1ca9, 0xff9e6b, 0x457d8b,
    0x92678b, 0x00cdbb, 0x9ccc04, 0x002e38, 0x96c57f, 0xcff6b4, 0x492818, 0x766e52,
    0x20370e, 0xe3d19f, 0x2e3c30, 0xb2eace, 0xf3bda4, 0xa24e3d, 0x976fd9, 0x8c9fa8,
    0x7c2b73, 0x4e5f37, 0x5d5462, 0x90956f, 0x6aa776, 0xdbcbf6, 0xda71ff, 0x987c95,
    0x52323c, 0xbb3c42, 0x584d39, 0x4fc15f, 0xa2b9c1, 0x79db21, 0x1d5958, 0xbd744e,
    0x160b00, 0x20221a, 0x6b8295, 0x00e0e4, 0x102401, 0x1b782a, 0xdaa9b5, 0xb0415d,
    0x859253, 0x97a094, 0x06e3c4, 0x47688c, 0x7c6755, 0x075c00, 0x7560d5, 0x7d9f00,
    0xc36d96, 0x4d913e, 0x5f4276, 0xfce4c8, 0x303052, 0x4f381b, 0xe5a532, 0x706690,
    0xaa9a92, 0x237363, 0x73013e, 0xff9079, 0xa79a74, 0x029bdb, 0xff0169, 0xc7d2e7,
    0xca8869, 0x80ffcd, 0xbb1f69, 0x90b0ab, 0x7d74a9, 0xfcc7db, 0x99375b, 0x00ab4d,
    0xabaed1, 0xbe9d91, 0xe6e5a7, 0x332c22, 0xdd587b, 0xf5fff7, 0x5d3033, 0x6d3800,
    0xff0020, 0xb57bb3, 0xd7ffe6, 0xc535a9, 0x260009, 0x6a8781, 0xa8abb4, 0xd45262,
    0x794b61, 0x4621b2, 0x8da4db, 0xc7c890, 0x6fe9ad, 0xa243a7, 0xb2b081, 0x181b00,
    0x286154, 0x4ca43b, 0x6a9573, 0xa8441d, 0x5c727b, 0x738671, 0xd0cfcb, 0x897b77,
    0x1f3f22, 0x4145a7, 0xda9894, 0xa1757a, 0x63243c, 0xadaaff, 0x00cde2, 0xddbc62,
    0x698eb1, 0x208462, 0x00b7e0, 0x614a44, 0x9bbb57, 0x7a5c54, 0x857a50, 0x766b7e,
    0x014833, 0xff8347, 0x7a8eba, 0x274740, 0x946444, 0xebd8e6, 0x646241, 0x373917,
    0x6ad450, 0x81817b, 0xd499e3, 0x979440, 0x011a12, 0x526554, 0xb5885c, 0xa499a5,
    0x03ad89, 0xb3008b, 0xe3c4b5, 0x96531f, 0x867175, 0x74569e, 0x617d9f, 0xe70452,
    0x067eaf, 0xa697b6, 0xb787a8, 0x9cff93, 0x311d19, 0x3a9459, 0x6e746e, 0xb0c5ae,
    0x84edf7, 0xed3488, 0x754c78, 0x384644, 0xc7847b, 0x00b6c5, 0x7fa670, 0xc1af9e,
    0x2a7fff, 0x72a58c, 0xffc07f, 0x9debdd, 0xd97c8e, 0x7e7c93, 0x62e674, 0xb5639e,
    0xffa861, 0xc2a580, 0x8d9c83, 0xb70546, 0x372b2e, 0x0098ff, 0x985975, 0x20204c,
    0xff6c60, 0x445083, 0x8502aa, 0x72361f, 0x9676a3, 0x484449, 0xced6c2, 0x3b164a,
    0xcca763, 0x2c7f77, 0x02227b, 0xa37e6f, 0xcde6dc, 0xcdfffb, 0xbe811a, 0xf77183,
    0xede6e2, 0xcdc6b4, 0xffe09e, 0x3a7271, 0xff7b59, 0x4e4e01, 0x4ac684, 0x8bc891,
    0xbc8a96, 0xcf6353, 0xdcde5c, 0x5eaadd, 0xf6a0ad, 0xe269aa, 0xa3dae4, 0x436e83,
    0x002e17, 0xecfbff, 0xa1c2b6, 0x50003f, 0x71695b, 0x67c4bb, 0x536eff, 0x5d5a48,
    0x890039, 0x969381, 0x371521, 0x5e4665, 0xaa62c3, 0x8d6f81, 0x2c6135, 0x410601,
    0x564620, 0xe69034, 0x6da6bd, 0xe58e56, 0xe3a68b, 0x48b176, 0xd27d67, 0xb5b268,
    0x7f8427, 0xff84e6, 0x435740, 0xeae408, 0xf4f5ff, 0x325800, 0x4b6ba5, 0xadceff,
    0x9b8acc, 0x885138, 0x5875c1, 0x7e7311, 0xfea5ca, 0x9f8b5b, 0xa55b54, 0x89006a,
    0xaf756f, 0x2a2000, 0x576e4a, 0x7f9eff, 0x7499a1, 0xffb550, 0x00011e, 0xd1511c,
    0x688151, 0xbc908a, 0x78c8eb, 0x8502ff, 0x483d30, 0xc42221, 0x5ea7ff, 0x785715,
    0x0cea91, 0xfffaed, 0xb3af9d, 0x3e3d52, 0x5a9bc2, 0x9c2f90, 0x8d5700, 0xadd79c,
    0x00768b, 0x337d00, 0xc59700, 0x3156dc, 0x944575, 0xecffdc, 0xd24cb2, 0x97703c,
    0x4c257f, 0x9e0366, 0x88ffec, 0xb56481, 0x396d2b, 0x56735f, 0x988376, 0x9bb195,
    0xa9795c, 0xe4c5d3, 0x9f4f67, 0x1e2b39, 0x664327, 0xafce78, 0x322edf, 0x86b487,
    0xc23000, 0xabe86b, 0x96656d, 0x250e35, 0xa60019, 0x0080cf, 0xcaefff, 0x323f61,
    0xa449dc, 0x6a9d3b, 0xff5ae4, 0x636a01, 0xd16cda, 0x736060, 0xffbaad, 0xd369b4,
    0xffded6, 0x6c6d74, 0x927d5e, 0x845d70, 0x5b62c1, 0x2f4a36, 0xe45f35, 0xff3b53,
    0xac84dd, 0x762988, 0x70ec98, 0x408543, 0x2c3533, 0x2e182d, 0x323925, 0x19181b,
    0x2f2e2c, 0x023c32, 0x9b9ee2, 0x58afad, 0x5c424d, 0x7ac5a6, 0x685d75, 0xb9bcbd,
    0x834357, 0x1a7b42, 0x2e57aa, 0xe55199, 0x316e47, 0xcd00c5, 0x6a004d, 0x7fbbec,
    0xf35691, 0xd7c54a, 0x62acb7, 0xcba1bc, 0xa28a9a, 0x6c3f3b, 0xffe47d, 0xdcbae3,
    0x5f816d, 0x3a404a, 0x7dbf32, 0xe6ecdc, 0x852c19, 0x285366, 0xb8cb9c, 0x0e0d00,
    0x4b5d56, 0x6b543f, 0xe27172, 0x0568ec, 0x2eb500, 0xd21656, 0xefafff, 0x682021,
    0x2d2011, 0xda4cff, 0x70968e, 0xff7b7d, 0x4a1930, 0xe8c282, 0xe7dbbc, 0xa68486,
    0x1f263c, 0x36574e, 0x52ce79, 0xadaaa9, 0x8a9f45, 0x6542d2, 0x00fb8c, 0x5d697b,
    0xccd27f, 0x94a5a1, 0x790229, 0xe383e6, 0x7ea4c1, 0x4e4452, 0x4b2c00, 0x620b70,
    0x314c1e, 0x874aa6, 0xe30091, 0x66460a, 0xeb9a8b, 0xeac3a3, 0x98eab3, 0xab9180,
    0xb8552f, 0x1a2b2f, 0x94ddc5, 0x9d8c76, 0x9c8333, 0x94a9c9, 0x392935, 0x8c675e,
    0xcce93a, 0x917100, 0x01400b, 0x449896, 0x1ca370, 0xe08da7, 0x8b4a4e, 0x667776,
    0x4692ad, 0x67bda8, 0x69255c, 0xd3bfff, 0x4a5132, 0x7e9285, 0x77733c, 0xe7a0cc,
    0x51a288, 0x2c656a, 0x4d5c5e, 0xc9403a, 0xddd7f3, 0x005844, 0xb4a200, 0x488f69,
    0x858182, 0xd4e9b9, 0x3d7397, 0xcae8ce, 0xd60034, 0xaa6746, 0x9e5585, 0xba6200
};

void LeaderElectionFaultTolerantParticle::expand(int label) {
    electionState = ElectionState::Crashed;
    electionPhase = ElectionPhase::Start;
    priorityEnabled = false;
    priority = 0;
    parentLabel = -1;
    mergeParticleLabel = -1;
    mergeParticlePathLabel = -1;

    if (treeColor == -1) {
        distinctColorsIndex = (distinctColorsIndex + 1) % distinctColors.size();
        treeColor = distinctColors[distinctColorsIndex];
    }

    AmoebotParticle::expand(label);
}

void LeaderElectionFaultTolerantParticle::push(int label) {
    electionState = ElectionState::Crashed;
    electionPhase = ElectionPhase::Start;
    priorityEnabled = false;
    priority = 0;
    parentLabel = -1;
    mergeParticleLabel = -1;
    mergeParticlePathLabel = -1;

    if (treeColor == -1) {
        distinctColorsIndex = (distinctColorsIndex + 1) % distinctColors.size();
        treeColor = distinctColors[distinctColorsIndex];
    }

    auto& nbr = nbrAtLabel(label);
    nbr.mergeParticleLabel = -1;
    nbr.mergeParticlePathLabel = -1;

    AmoebotParticle::push(label);
}

void LeaderElectionFaultTolerantParticle::contractTail() {
    electionState = ElectionState::Crashed;
    electionPhase = ElectionPhase::Start;
    priorityEnabled = false;
    priority = 0;
    parentLabel = -1;
    mergeParticleLabel = -1;
    mergeParticlePathLabel = -1;

    if (treeColor == -1) {
        distinctColorsIndex = (distinctColorsIndex + 1) % distinctColors.size();
        treeColor = distinctColors[distinctColorsIndex];
    }

    for (const int label : tailLabels()) {
        if (hasNbrAtLabel(label)) {
            auto& nbr = nbrAtLabel(label);
            if ((nbr.electionState == ElectionState::Child) && pointsAtMe(nbr, nbr.parentLabel)) {
                nbr.electionState = ElectionState::Crashed;
                nbr.electionPhase = ElectionPhase::Start;
                nbr.parentLabel = -1;
            }
        }
    }

    AmoebotParticle::contractTail();
}

LeaderElectionFaultTolerantSystem::LeaderElectionFaultTolerantSystem(int numParticles, double holeProb,
                                                                     QString mode) {
    Q_ASSERT(mode == "h" || mode == "s" || mode == "t1" || mode == "t2" ||
             mode == "l");
    Q_ASSERT(numParticles > 0);
    Q_ASSERT(0 <= holeProb && holeProb <= 1);

    _seedOrientation = randDir();
    insert(new LeaderElectionFaultTolerantParticle(Node(0, 0), -1, _seedOrientation, *this,
                                                   LeaderElectionFaultTolerantParticle::State::Seed, mode));
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
            insert(new LeaderElectionFaultTolerantParticle(randomCandidate, -1, 0, *this,
                                                           LeaderElectionFaultTolerantParticle::State::Idle,
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

bool LeaderElectionFaultTolerantSystem::hasTerminated() const {
#ifdef QT_DEBUG
    if (!isConnected(particles)) {
        return true;
    }
#endif

    for (auto p : particles) {
        auto hp = dynamic_cast<LeaderElectionFaultTolerantParticle*>(p);
        if (!hp->isFinished() || (hp->pathToRootState != LeaderElectionFaultTolerantParticle::PathToRootState::Valid)) {
            return false;
        }
    }

    return true;
}

void LeaderElectionFaultTolerantSystem::nuke() {
    for (auto p : particles) {
        auto hp = dynamic_cast<LeaderElectionFaultTolerantParticle*>(p);
        if (hp->state != LeaderElectionFaultTolerantParticle::State::Seed) {
            crashParticleAt(hp->head);
        }
    }
}

std::set<QString> LeaderElectionFaultTolerantSystem::getAcceptedModes() {
    std::set<QString> set = {"h"};
    return set;
}

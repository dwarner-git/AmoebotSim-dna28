/* Copyright (C) 2022 Daniel Warner.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "alg/chirality_comparison.h"

#include <QtGlobal>

ChiralityComparisonParticle::ChiralityComparisonParticle(const Node head,
                                                                         const int globalTailDir,
                                                                         const int orientation,
                                                                         AmoebotSystem& system,
                                                                         State state)
    : AmoebotParticle(head, globalTailDir, orientation, system),
      state(state),
      hasCompared({}),
      isBoundaryParticle(false)
{
}

void ChiralityComparisonParticle::activate() {
    if (state == State::Done) {
        return;
    }

    for (int label = 0; label < 6; label++) {
        if (!hasNbrAtLabel(label)) {
            isBoundaryParticle = true;
        }
    }

    for (int q_label = 0; q_label < 6; q_label++) {
        if (!hasCompared[q_label] && hasNbrAtLabel(q_label)) {
            if (!hasNbrAtLabel((q_label + 1) % 6) || !hasNbrAtLabel((q_label + 5) % 6)) {
                // there exists a common neighbouring empty node
                hasCompared[q_label] = true;
            } else {
                ChiralityComparisonParticle& q = nbrAtLabel(q_label);
                int r_label = (labelToNbrLabel(q_label) + 1) % 6;
                if (q.hasCompared[r_label])
                {
                    hasCompared[q_label] = true;
                    q.hasCompared[labelToNbrLabel(q_label)] = true;
                }
            }
        }
    }

    int done = true;
    for (int label = 0; label < 6; label++) {
        if (!hasCompared[label] && hasNbrAtLabel(label)) {
            done = false;
            break;
        }
    }

    if (done) {
        state = State::Done;
    }
}

int ChiralityComparisonParticle::particleColor() const {
    if (state == State::Done) {
        return 0x00ff00; // bright green/lime
    }

    return 0x000000; // black
}

int ChiralityComparisonParticle::headMarkColor() const {
    // color pool:
    // raspberry: 0xff0077, turquoise:0x00ff77, bright magenta/fuchsia:0xff00ff, spring green:0x77ff00, yellow: 0xffff00, cyan: 0x00ffff, orange: 0xff7700

    if (isBoundaryParticle) {
        return 0xff0000; // red
    }

    return 0x000000; // black
}

int ChiralityComparisonParticle::tailMarkColor() const {
    return headMarkColor();
}

std::array<std::array<std::array<VisFlagStates, 2>, 2>, 6> ChiralityComparisonParticle::getSafeFlags() const {
  std::array<std::array<std::array<VisFlagStates, 2>, 2>, 6> flags = {};

  for (int localDir = 0; localDir < 6; localDir++) {
      int globalDir = localToGlobalDir(localDir);
      VisFlagStates visFlagState = hasCompared[localDir] ? VisFlagStates::SafeState_Safe: VisFlagStates::SafeState_Undetermined;
      flags[globalDir][0][0] = visFlagState;
  }

  return flags;
}

QString ChiralityComparisonParticle::inspectionText() const {
    QString text;
    text += "head: (" + QString::number(head.x) + ", " + QString::number(head.y) +
            ")\n";
    text += "orientation: " + QString::number(orientation) + "\n";
    text += "globalTailDir: " + QString::number(globalTailDir) + "\n";
    text += "state: ";
    text += [this](){
        switch(state) {
        case State::Idle:   return "Idle";
        case State::Done:   return "Done";
        default:            return "no state";
        }
    }();
    text += "\n";

    return text;
}

ChiralityComparisonParticle& ChiralityComparisonParticle::nbrAtLabel(int label) const {
    return AmoebotParticle::nbrAtLabel<ChiralityComparisonParticle>(label);
}

int ChiralityComparisonParticle::labelOfFirstNbrInState(
        std::initializer_list<State> states, int startLabel, bool ignoreErrorParticles) const {
    auto prop = [&](const ChiralityComparisonParticle& p) {
        for (auto state : states) {
            if (p.state == state) {
                return true;
            }
        }
        return false;
    };

    return labelOfFirstNbrWithProperty<ChiralityComparisonParticle>(prop, startLabel, ignoreErrorParticles);
}

bool ChiralityComparisonParticle::hasNbrInState(std::initializer_list<State> states) const {
    return labelOfFirstNbrInState(states) != -1;
}

bool ChiralityComparisonParticle::isInState(std::initializer_list<State> states) const {
    for (auto _state : states) {
        if (_state == state) {
            return true;
        }
    }

    return false;
}

ChiralityComparisonSystem::ChiralityComparisonSystem(int numParticles, double holeProb,
                                                                     QString mode) {
    Q_ASSERT(mode == "h" || mode == "s" || mode == "t1" || mode == "t2" ||
             mode == "l");
    Q_ASSERT(numParticles > 0);
    Q_ASSERT(0 <= holeProb && holeProb <= 1);

    // Insert first particle at (0,0).
    insert(new ChiralityComparisonParticle(Node(0, 0), -1, randDir(), *this,
                                                   ChiralityComparisonParticle::State::Idle));
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
            insert(new ChiralityComparisonParticle(randomCandidate, -1, randDir(), *this,
                                                           ChiralityComparisonParticle::State::Idle));

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

bool ChiralityComparisonSystem::hasTerminated() const {
#ifdef QT_DEBUG
    if (!isConnected(particles)) {
        return true;
    }
#endif

    for (auto p : particles) {
        auto hp = dynamic_cast<ChiralityComparisonParticle*>(p);
        if ((hp->state != ChiralityComparisonParticle::State::Done)) {
            return false;
        }
    }

    return true;
}

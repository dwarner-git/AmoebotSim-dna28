/* Copyright (C) 2021 Joshua J. Daymude, Robert Gmyr, Kristian Hinnenthal and Daniel Warner.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp. */

#include "core/amoebotsystem.h"

#include <QDateTime>
#include <QtGlobal>

#include "core/amoebotparticle.h"

AmoebotSystem::AmoebotSystem() {
  _counts.push_back(new Count("# Rounds"));
  _counts.push_back(new Count("# Activations"));
  _counts.push_back(new Count("# Moves"));
  _counts.push_back(new Count("# Crashes"));
}

AmoebotSystem::~AmoebotSystem() {
  for (auto p : particles) {
    delete p;
  }
  particles.clear();

  for (auto t : objects) {
    delete t;
  }
  objects.clear();

  for (auto c : _counts) {
    delete c;
  }

  for (auto m : _measures) {
    delete m;
  }
}

void AmoebotSystem::activate(int errorRate, int errorRounds) {
    if (particles.size() > 0) {
        int randomParticleIndex = randInt(0, particles.size());
    	AmoebotParticle* particle = particles.at(randomParticleIndex);

    	int rounds = getCount("# Rounds")._value;
    	double errorValue = randDouble(0, 100);

    	if (errorValue <= errorRate && rounds % errorRounds == 0) {
        	registerActivation(particle);
        	registerCrash();
        	particle->crash();
    	} else {
        	registerActivation(particle);
        	particle->activate();
    	}
	}
}

void AmoebotSystem::activateParticleAt(Node node) {
  auto it = particleMap.find(node);
  if (it != particleMap.end()) {
    registerActivation(it->second);
    it->second->activate();
  }
}

void AmoebotSystem::crashParticleAt(Node node) {
  auto it = particleMap.find(node);
  if (it != particleMap.end()) {
      it->second->crash();
      registerActivation(it->second);
      registerCrash();
  }
}

unsigned int AmoebotSystem::size() const {
  return particles.size();
}

unsigned int AmoebotSystem::numObjects() const {
  return objects.size();
}

const Particle& AmoebotSystem::at(int i) const {
  return *particles.at(i);
}

const std::deque<Object*>& AmoebotSystem::getObjects() const {
  return objects;
}

void AmoebotSystem::insert(AmoebotParticle* particle) {
  Q_ASSERT(particleMap.find(particle->head) == particleMap.end());
  Q_ASSERT(objectMap.find(particle->head) == objectMap.end());
  Q_ASSERT(!particle->isExpanded() ||
           particleMap.find(particle->tail()) == particleMap.end());

  particles.push_back(particle);
  particleMap[particle->head] = particle;
  if (particle->isExpanded()) {
    particleMap[particle->tail()] = particle;
  }
}

void AmoebotSystem::insert(Object* object) {
  Q_ASSERT(objectMap.find(object->_node) == objectMap.end());
  Q_ASSERT(particleMap.find(object->_node) == particleMap.end());

  objects.push_back(object);
  objectMap[object->_node] = object;
}

void AmoebotSystem::remove(AmoebotParticle* particle) {
  particles.erase(std::remove(particles.begin(), particles.end(), particle),
                  particles.end());
  auto it = particleMap.begin();
  while (it != particleMap.end()) {
    if (it->second == particle) {
      it = particleMap.erase(it);
    } else {
      it++;
    }
  }
  activatedParticles.erase(particle);

  delete particle;
}

void AmoebotSystem::registerMovement(unsigned int numMoves) {
  getCount("# Moves").record(numMoves);
}

void AmoebotSystem::registerCrash(unsigned int numCrashes) {
  getCount("# Crashes").record(numCrashes);
}

void AmoebotSystem::registerActivation(AmoebotParticle* particle) {
  getCount("# Activations").record();
  activatedParticles.insert(particle);
  if (activatedParticles.size() == particles.size()) {
    registerRound();
    activatedParticles.clear();
  }
}

void AmoebotSystem::registerRound() {
  for (const auto& c : _counts) {
    c->_history.push_back(c->_value);
  }
  for (const auto& m : _measures) {
    if (getCount("# Rounds")._value % m->_freq == 0) {
      m->_history.push_back(m->calculate());
    }
  }
  getCount("# Rounds").record();
}

const std::vector<Count*>& AmoebotSystem::getCounts() const {
  return _counts;
}

const std::vector<Measure*>& AmoebotSystem::getMeasures() const {
  return _measures;
}

Count& AmoebotSystem::getCount(QString name) const {
  for (const auto& c : _counts) {
    if (QString::compare(c->_name, name) == 0) {
      return *c;
    }
  }
  Q_ASSERT(false);  // Requested count does not exist.
}

Measure& AmoebotSystem::getMeasure(QString name) const {
  for (const auto& m : _measures) {
    if (QString::compare(m->_name, name) == 0) {
      return *m;
    }
  }
  Q_ASSERT(false);  // Requested measure does not exist.
}


const QString AmoebotSystem::metricsAsJSON() const {
  QString json = "{\"title\" : \"AmoebotSim Metrics JSON\", ";
  json += "\"datetime\" : \"" +
          QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss") + "\", ";
  json += "\"algorithm\" : \"???\", ";
  json += "\"counts\" : [";
  for (const auto& c : _counts) {
    json += "{\"name\" : \"" + c->_name + "\", ";
    json += "\"history\" : [";
    for (auto val : c->_history) {
      json += QString::number(val) += ", ";
    }
    if (!c->_history.empty()) {
      json.chop(2);  // Remove the last ", ".
    }
    json += "]}, ";
  }
  if (!_counts.empty()) {
    json.chop(2);  // Remove the last ", ".
  }
  json += "], \"measures\" : [";
  for (const auto& m : _measures) {
    json += "{\"name\" : \"" + m->_name + "\", ";
    json += "\"frequency\" : " + QString::number(m->_freq) + ", ";
    json += "\"history\" : [";
    for (auto val : m->_history) {
      json += QString::number(val) += ", ";
    }
    if (!m->_history.empty()) {
      json.chop(2);  // Remove the last ", ".
    }
    json += "]}, ";
  }
  if (!_measures.empty()) {
    json.chop(2);  // Remove the last ", ".
  }
  json += "]}";
  return json;
}

int AmoebotSystem::seedOrientation() const {
    return _seedOrientation;
}

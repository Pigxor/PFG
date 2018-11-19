#pragma once
#ifndef _DYNAMIC_H_
#define _DYNAMIC_H_

#include "GameObject.h"
#include "Utility.h"

class Dynamic : public GameObject
{
public:
  Dynamic();
  ~Dynamic();

  virtual void Update(float deltaTs);
  void SetVelocity(glm::vec3 vel) { _velocity = vel; }
  glm::vec3 GetVelocity() { return _velocity; }
  void SetLanded(int landed) { _landed = landed; }
  void StartSimulation(bool start) { _start = start; }
  void ClearForces() { _force = glm::vec3(0, 0, 0); }
  void AddForce(glm::vec3 newForce);
  void AddTorque(glm::vec3 newTorque) { _torque += newTorque; };
  void ClearTorque() {_torque = glm::vec3(0.0f, 0.0f, 0.0f);}
  void ComputeInertiaTensor();
  void Euler(float deltaTs);
  void SetAcc(glm::vec3 accel) { acc = accel; }
  void SetMass(float M) { mass = M; }
  float GetMass() { return mass; }
  void CollisionResponse(float deltaTs);
  void RungeKutta2(float deltaTs);
  void RungeKutta4(float deltaTs);
  glm::mat4 GetRotation() { return _orientation; }
  //void SetScale(glm::vec3 scale) { _scale = scale; }
  //glm::vec3 GetPosition() { return _position; }
  //void SetPosition(glm::vec3 pos) { _position = pos; }

private:

  float mass;
  glm::vec3 prevPos;
  glm::vec3 _force;
  void UpdateModelMatrix();
  bool _start = false;
  glm::vec3 _velocity;
  int _landed = 0;
  float _bounce = 1.0f;
  glm::vec3 acc;
  glm::mat4 _orientation;
  glm::vec3 _torque;
  glm::vec3 _angularVel;
  glm::mat3 _inertiaTensorInv;
  glm::mat3 _inertiaTensor;
  glm::vec3 _angularMom;
  glm::mat3 _bodyInertiaTensorInv;
  glm::mat3 _R;
  glm::mat3 _SkewAngVel;
  glm::mat3 bodyTensor;

  //glm::vec3 _scale;
  //glm::vec3 _position;

};


#endif // !_DYNAMIC_H_

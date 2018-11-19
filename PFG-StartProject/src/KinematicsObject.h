#ifndef _KinematicsObject_H_
#define _KinematicsObject_H_

#include "GameObject.h"

class KinematicsObject : public GameObject
{
public :
  KinematicsObject();
  ~KinematicsObject();

  void Update(float deltaTs,GameObject* level);
  void SetVelocity(glm::vec3 vel) { _velocity = vel; }
  int GetLanded() { return _landed; }
  void SetLanded(int landed) { _landed = landed; }
  void StartSimulation(bool start) { _start = start; }

  //glm::mat4 GetRotation() { return _orientation; }
  //void SetScale(glm::vec3 scale) { _scale = scale; }
  //glm::vec3 GetPosition() { return _position; }
  //void SetPosition(glm::vec3 pos) { _position = pos; }

private:

  void UpdateModelMatrix();
  bool _start = false;
  glm::vec3 _velocity;
  int _landed = 0;
  //glm::mat4 _orientation;
  //glm::vec3 _scale;
  //glm::vec3 _position;
 
};
#endif
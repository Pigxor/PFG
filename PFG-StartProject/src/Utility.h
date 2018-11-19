#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>


namespace PFG
{
  float DistToPlane(const glm::vec3& n, const glm::vec3& p, const glm::vec3& q);
  bool MovSpherePlaneColl(const glm::vec3& n, const glm::vec3& c0, const glm::vec3& c1, const glm::vec3& q, float r, glm::vec3 &ci);
  bool MovSpherePlaneColl2(glm::vec3 n, glm::vec3 c0, glm::vec3 c1, glm::vec3 q, float r, glm::vec3 &ci);
  bool Sphere2SphereColl(glm::vec3 c0, glm::vec3 c1, glm::vec3 q, float r1, float r2, glm::vec3 &cp);

}
#include "Utility.h"
namespace PFG
{
  float DistToPlane(const glm::vec3 & n, const glm::vec3 & p, const glm::vec3 & q)
  {
    float d = glm::dot((p - q), n);
    return d;
  }
  bool MovSpherePlaneColl(const glm::vec3 & n, const glm::vec3 & c0, const glm::vec3 & c1, const glm::vec3 & q, float r, glm::vec3 & ci)
  {
    return false;
  }
  bool MovSpherePlaneColl2(glm::vec3 n, glm::vec3 c0, glm::vec3 c1, glm::vec3 q, float r, glm::vec3 & ci)
  {
    float t;
    float d0 = DistToPlane(n, c0, q);
    float d1 = DistToPlane(n, c1, q);

    if (glm::abs(d0) <= r)
    {
      ci = c0;
      t = 0.0f;
      return true;
    }
    if (d0 > r && d1 < r)
    {
      t = (d0 - r) / (d0 - d1);
      return true;
    }
    return false;
  }
  bool Sphere2SphereColl(glm::vec3 c0, glm::vec3 c1, glm::vec3 q, float r1, float r2, glm::vec3 & cp)
  {
    float d = glm::length(c0 - c1);
    glm::vec3 n;

    if (d <= (r1 + r2))
    {
      n = glm::normalize(c0 - c1);
      cp = r1 * n;
      return true;
    }
    return false;
  }
}
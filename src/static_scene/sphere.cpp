#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // TODO:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  return false;

}

bool Sphere::intersect(const Ray& r) const {

  // TODO:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
	Vector3D L = o - r.o;
	double tca = dot(L,r.d);
	double d2 = dot(L,L) - tca*tca;
	return r2 > d2;

}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

	if(!intersect(r)) return false;


	Vector3D L = r.o - o;
	double a = dot(r.d,r.d);
	double b = 2 * dot(r.d,L);
	//cout<<b<<" ";
	double c = dot(L,L) - r2;

	double t0,t1;

	if(!solveQuadratic(a,b,c,t0,t1))
		return false;
	if(t0 > t1) std::swap(t0,t1);

	if(t0 < 0) t0 = t1;
	if(t0 < 0)	return false;
	Vector3D v = r.o + t0 * r.d;
	//cout<<L.norm() - this->r<<" ";
	//cout<<v.norm()<<" ";
	//cout<<o.norm()<<" ";
	//cout<<r.o.norm()<<endl;
	i->t = t0;
	i->primitive = this;
	i->bsdf = this->get_bsdf();
	i->n = this->normal(r.o + r.d*t0);
	return true;

}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CMU462

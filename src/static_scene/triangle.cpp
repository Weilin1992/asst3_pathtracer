#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 { namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3) :
    mesh(mesh), v1(v1), v2(v2), v3(v3) { }

BBox Triangle::get_bbox() const {
  
  // TODO: 
  // compute the bounding box of the triangle
	Vector3D Tri_V1 = mesh->positions[v1];
	Vector3D Tri_V2 = mesh->positions[v2];
	Vector3D Tri_V3 = mesh->positions[v3];

	double x1 = Tri_V1.x; double y1 = Tri_V1.y; double z1 = Tri_V1.z;
	double x2 = Tri_V2.x; double y2 = Tri_V2.y; double z2 = Tri_V2.z;
	double x3 = Tri_V3.x; double y3 = Tri_V3.y; double z3 = Tri_V3.z;

	double inte = (x1 > x2? x2:x1);
	double minX =  inte > x3 ? x3 : inte;
	inte = (y1 > y2? y2:y1);
	double minY =  inte > y3 ? y3 : inte;
	inte = z1 > z2 ? z2 : z1;
	double minZ = inte > z3 ? z3 : inte;

	inte = (x1 < x2? x2:x1);
	double maxX =  inte < x3 ? x3 : inte;
	inte = (y1 < y2? y2:y1);
	double maxY =  inte < y3 ? y3 : inte;
	inte = z1 < z2 ? z2 : z1;
	double maxZ = inte < z3 ? z3 : inte;


	BBox bbox_t = BBox(minX, minY, minZ,
										 maxX, maxY, maxZ);

	return bbox_t;

}

bool Triangle::intersect(const Ray& r) const {
  
  // TODO: implement ray-triangle intersection

//	Vector3D n = mesh->normals[v1] + mesh->normals[v2] + mesh->normals[v3];
//	n.normalize();
//	if(dot(r.d,n) > 0) return false;

	Vector3D e1 = mesh->positions[v2] - mesh->positions[v1];
	Vector3D e2 = mesh->positions[v3] - mesh->positions[v1];
	Vector3D s = r.o - mesh->positions[v1];
	Vector3D e1xd = cross(e1,r.d);
	double e1de2 =dot(e1xd,e2);

	if(abs(e1de2) <= 1e-15) return false;

	Vector3D se2 = cross(s,e2);
	Vector3D e1d = cross(e1,r.d);
	double u = -dot(se2,r.d)/e1de2;
	double v = dot(e1d,s)/e1de2;

	if(v < 0 || u < 0 || (v + u) > 1) return false;

	double t = -dot(se2,e1)/e1de2;

	if(r.max_t < t || r.min_t > t) return false;

  return true;
}

bool Triangle::intersect(const Ray& r, Intersection *isect) const {

	Vector3D n = mesh->normals[v1] + mesh->normals[v2] + mesh->normals[v3];
	n.normalize();

	Vector3D e1 = mesh->positions[v2] - mesh->positions[v1];
	Vector3D e2 = mesh->positions[v3] - mesh->positions[v1];
	Vector3D s = r.o - mesh->positions[v1];

	Vector3D e1xd = cross(e1,r.d);
	double e1de2 =dot(e1xd,e2);

	if(abs(e1de2) <= 1e-15) return false;

	Vector3D se2 = cross(s,e2);
	Vector3D e1d = cross(e1,r.d);
	double u = -dot(se2,r.d)/e1de2;
	double v = dot(e1d,s)/e1de2;

	if(v < 0 || u < 0 || (v + u) > 1) return false;

	double t = -dot(se2,e1)/e1de2;

	if(r.max_t < t || r.min_t > t)	return false;

	if(dot(r.d,n) > 0){
		n = -n;
		isect->isfronthit = false;
	}else{
		isect->isfronthit = true;
	}

	isect->bsdf = this->mesh->get_bsdf();
	isect->n = n;
	isect->primitive = this;
	isect->t = t;

	return true;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x,
             mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x,
             mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x,
             mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}



} // namespace StaticScene
} // namespace CMU462

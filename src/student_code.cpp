#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
      std::vector<Vector2D> vect = std::vector<Vector2D>();
      if (points.size() == 1) {
          vect.push_back(points[0]);
      }
      else {
          for (int i = 0; i < points.size() - 1; i++) {
              vect.push_back((1 - t) * points[i] + t * points[i + 1]);
          }
      }
      return vect;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> vect;
      if (points.size() == 1) {
          vect.push_back(points[0]);
      }
      else {
          for (int i = 0; i < points.size() - 1; i++) {
              vect.push_back((1 - t) * points[i] + t * points[i + 1]);
          }
      }
      return vect;

  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
        std::vector<Vector3D> inter = points;
        while (inter.size() > 1) {
            inter = BezierPatch::evaluateStep(inter, t);
        }
        return inter[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
      std::vector<Vector3D> moving;
      for (int i = 0; i < controlPoints.size(); i++) {
          moving.push_back(BezierPatch::evaluate1D(controlPoints[i], u));
      }
      return BezierPatch::evaluate1D(moving, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      std::vector<Vector3D> trivert = std::vector<Vector3D>();
      Vector3D awnorm;
      float area;
      Vector3D a, b;
      HalfedgeCIter h = halfedge();
      FaceCIter f = h->face();

      do {
          trivert.clear();
          trivert.push_back(h->vertex()->position);
          trivert.push_back(h->next()->vertex()->position);
          trivert.push_back(h->next()->next()->vertex()->position);

          a = trivert[1] - trivert[0];
          b = trivert[2] - trivert[0];
          area = abs(cross(a, b).norm()) / 2.0;
          awnorm += f->normal() * area;
          h = h->twin()->next();
          f = h->face();
      } while (h != halfedge());
     
      awnorm.normalize();
      return awnorm;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    
      if (e0->isBoundary()) {
          return e0;
      }

      // credit to http://15462.courses.cs.cmu.edu/fall2015content/misc/HalfedgeEdgeOpImplementationGuide.pdf
      // collecting elements
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      // reassigning elements
      // half-edges
      h0->setNeighbors(h1, h3, v3, e0, f0);
      h1->setNeighbors(h2, h7, v2, e2, f0);
      h2->setNeighbors(h0, h8, v0, e3, f0);
      h3->setNeighbors(h4, h0, v2, e0, f1);
      h4->setNeighbors(h5, h9, v3, e4, f1);
      h5->setNeighbors(h3, h6, v1, e1, f1);

      h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());

      // vertices
      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h3;
      v3->halfedge() = h0;
      
      // edges
      e0->halfedge() = h0;
      e1->halfedge() = h5;
      e2->halfedge() = h1;
      e3->halfedge() = h2;
      e4->halfedge() = h4;

      // faces
      f0->halfedge() = h0;
      f1->halfedge() = h3;

      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if (e0->isBoundary()) {
          return e0->halfedge()->vertex();
      }
      
      // collecting elements
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      // allocating new elements
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      HalfedgeIter h12 = newHalfedge();
      HalfedgeIter h13 = newHalfedge();
      HalfedgeIter h14 = newHalfedge();
      HalfedgeIter h15 = newHalfedge();

      VertexIter v4 = newVertex();
      v4->position = (v0->position + v1->position) / 2.0;
      v4->isNew = true;

      EdgeIter e5 = newEdge();
      e5->isNew = false;
      EdgeIter e6 = newEdge();
      e6->isNew = true;
      EdgeIter e7 = newEdge();
      e7->isNew = true;

      FaceIter f2 = newFace();
      FaceIter f3 = newFace();

      // reassigning elements
      // half-edges
      h0->setNeighbors(h15, h3, v0, e0, f0);
      h1->setNeighbors(h10, h6, v1, e1, f2);
      h2->setNeighbors(h0, h7, v2, e2, f0);
      h3->setNeighbors(h4, h0, v4, e0, f1);
      h4->setNeighbors(h14, h8, v0, e3, f1);
      h5->setNeighbors(h12, h9, v3, e4, f3);

      h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());

      h10->setNeighbors(h11, h15, v2, e6, f2);
      h11->setNeighbors(h1, h12, v4, e5, f2);
      h12->setNeighbors(h13, h11, v1, e5, f3);
      h13->setNeighbors(h5, h14, v4, e7, f3);
      h14->setNeighbors(h3, h13, v3, e7, f1);
      h15->setNeighbors(h2, h10, v4, e6, f0);

      // vertices
      v0->halfedge() = h0;
      v1->halfedge() = h1;
      v2->halfedge() = h2;
      v3->halfedge() = h5;
      v4->halfedge() = h3;

      // edges
      e0->halfedge() = h0;
      e1->halfedge() = h1;
      e2->halfedge() = h2;
      e3->halfedge() = h4;
      e4->halfedge() = h5;
      e5->halfedge() = h11;
      e6->halfedge() = h10;
      e7->halfedge() = h13;

      // faces
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      f2->halfedge() = h1;
      f3->halfedge() = h5;

      return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
      VertexIter a, b, c, d;
      HalfedgeIter x;
      int deg;
      float u;
      std::vector<Vector3D> neighbors = std::vector<Vector3D>();
      Vector3D neighborsum;

      // preprocessing by marking old vertices and edges, and calculating updated positions
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          e->isNew = false;
          a = e->halfedge()->vertex();
          b = e->halfedge()->twin()->vertex();
          c = e->halfedge()->next()->next()->vertex();
          d = e->halfedge()->twin()->next()->next()->vertex();
          e->newPosition = (3.0 / 8.0) * (a->position + b->position) + (1.0 / 8.0) * (c->position + d->position);
      }
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->isNew = false;
          neighbors.clear();
          neighborsum = Vector3D();
          x = v->halfedge();
          do {
              neighbors.push_back(x->twin()->vertex()->position);
              x = x->twin()->next();
          } while (x != v->halfedge());

          deg = neighbors.size();
          for (int i = 0; i < deg; i++) {
              neighborsum += neighbors[i];
          }

          if (deg == 3) {
              u = 3.0 / 16.0;
          }
          else {
              u = 3.0 / (8.0 * (double)deg);
          }
          v->newPosition = (1.0 - ((double)deg * u)) * v->position + u * neighborsum;
      }

      // edge splits
      EdgeIter e = mesh.edgesBegin();
      while (e != mesh.edgesEnd()) {
          if (e->isNew == false && e->halfedge()->vertex()->isNew == false && e->halfedge()->twin()->vertex()->isNew == false) {
              VertexIter v = mesh.splitEdge(e);
              v->newPosition = e->newPosition;
          }
          e++;
      }


      // flipping edges
      e = mesh.edgesBegin();
      while (e != mesh.edgesEnd()) {
          if (e->isNew == true && (e->halfedge()->vertex()->isNew != e->halfedge()->twin()->vertex()->isNew)) {
              mesh.flipEdge(e);
          }
          e++;
      }

      // reassign vertices
      VertexIter v = mesh.verticesBegin();
      while (v != mesh.verticesEnd()) {
          v->position = v->newPosition;
          v++;
      }
  }
}
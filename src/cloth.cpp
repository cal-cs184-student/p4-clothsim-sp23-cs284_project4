#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      vector<int> coordinate = {j, i};
      if (orientation == HORIZONTAL) {
        Vector3D position(width / num_width_points * j, 1, height / num_height_points * i);
        PointMass point_mass(position, find(pinned.begin(), pinned.end(), coordinate) != pinned.end());
        point_masses.emplace_back(point_mass);
      }
      else if (orientation == VERTICAL) {
        Vector3D position(width / num_width_points * j, height / num_height_points * i, ((double) rand() / RAND_MAX - 0.5) / 500);
        PointMass point_mass(position, find(pinned.begin(), pinned.end(), coordinate) != pinned.end());
        point_masses.emplace_back(point_mass);
      }
    }
  }

  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      int index = i * num_width_points + j;

      if (j >= 1) {
        int left_index = i * num_width_points + (j - 1);
        Spring spring(&point_masses[index], &point_masses[left_index], STRUCTURAL);
        springs.emplace_back(spring);
      }

      if (i >= 1) {
        int up_index = (i - 1) * num_width_points + j;
        Spring spring(&point_masses[index], &point_masses[up_index], STRUCTURAL);
        springs.emplace_back(spring);
      }

      if (i >= 1 && j >= 1) {
        int top_left_index = (i - 1) * num_width_points + (j - 1);
        Spring spring(&point_masses[index], &point_masses[top_left_index] , SHEARING);
        springs.emplace_back(spring);
      }

      if (i >= 1 && j < num_width_points - 1) {
        int top_right_index = (i - 1) * num_width_points + (j + 1);
        Spring spring(&point_masses[index], &point_masses[top_right_index] , SHEARING);
        springs.emplace_back(spring);
      }

      if (i >= 2) {
        int up_two_index = (i - 2) * num_width_points + j;
        Spring spring(&point_masses[index], &point_masses[up_two_index] , BENDING);
        springs.emplace_back(spring);
      }

      if (j >= 2) {
        int left_two_index = i * num_width_points + (j - 2);
        Spring spring(&point_masses[index], &point_masses[left_two_index] , BENDING);
        springs.emplace_back(spring);
      }
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      int index = i * num_width_points + j;
      point_masses[index].forces = Vector3D(0,0,0);
    }
  }

  // TODO (Part 2): Compute total force acting on each point mass.

  Vector3D external_force(0,0,0);
  for (int i = 0; i < external_accelerations.size(); i++) {
    external_force += external_accelerations[i] * mass;
  }
  for (int i = 0; i < point_masses.size(); i++) {
    point_masses[i].forces = external_force;
  }

  for (int i = 0; i < springs.size(); i++) {
    Spring spring = springs[i];
    Vector3D direction = spring.pm_a -> position - spring.pm_b -> position;
    direction.normalize();
    double distance = (spring.pm_a -> position - spring.pm_b -> position).norm();

    if (cp -> enable_structural_constraints && spring.spring_type == STRUCTURAL) {
          Vector3D spring_force = direction * cp -> ks * (distance - spring.rest_length);
          spring.pm_a -> forces -= spring_force;
          spring.pm_b -> forces += spring_force;
    }

    if (cp -> enable_shearing_constraints && spring.spring_type == SHEARING) {
          Vector3D spring_force = direction * cp -> ks * (distance - spring.rest_length);
          spring.pm_a -> forces -= spring_force;
          spring.pm_b -> forces += spring_force;
    }

    if (cp -> enable_bending_constraints && spring.spring_type == BENDING) {
        Vector3D spring_force = direction * 0.2 * cp -> ks * (distance - spring.rest_length);
        spring.pm_a -> forces -= spring_force;
        spring.pm_b -> forces += spring_force;
    }
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions

  for (int i = 0; i < point_masses.size(); i++) {
    PointMass* point_mass = &point_masses[i];

    if (!point_mass -> pinned) {
      Vector3D current_position = point_mass -> position;
      double d = cp -> damping / 100.;
      Vector3D a = point_mass -> forces / mass;
      point_mass -> position += (1 - d) * (point_mass -> position - point_mass -> last_position) + (a * delta_t * delta_t);
      point_mass-> last_position = current_position;
    }
    // point_mass -> forces = Vector3D(0);
  }
  

  // TODO (Part 4): Handle self-collisions.


  // TODO (Part 3): Handle collisions with other primitives.
  for (int i = 0; i < (*collision_objects).size(); i++) {
    CollisionObject* object = (*collision_objects)[i];
    for (int j = 0; j < point_masses.size(); j++) {
      PointMass* point_mass = &point_masses[j];
       object -> collide(*point_mass);
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
    Spring spring = springs[i];
    Vector3D direction = spring.pm_a -> position - spring.pm_b -> position;
    direction.normalize();
    double distance = (spring.pm_a -> position - spring.pm_b -> position).norm();
    
    if (distance > 1.1 * spring.rest_length) {
      if (!spring.pm_a -> pinned && !spring.pm_b -> pinned) {
        spring.pm_a -> position -= 0.5 * direction * (distance - 1.1 * spring.rest_length);
        spring.pm_b -> position += 0.5 * direction * (distance - 1.1 * spring.rest_length);
      } 
  
      else if (spring.pm_a -> pinned && !spring.pm_b -> pinned) {
        spring.pm_b -> position += direction * (distance - 1.1 * spring.rest_length);
      }

      else if (!spring.pm_a -> pinned && spring.pm_b -> pinned) {
        spring.pm_a -> position -= direction * (distance - 1.1 * spring.rest_length);
      } 
    }
    
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  return 0.f; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}

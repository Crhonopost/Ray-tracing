#include "Mesh.h"
#include <iostream>
#include <fstream>

void Mesh::loadOFF (const std::string & filename) {
    std::ifstream in (filename.c_str ());
    if (!in)
        exit (EXIT_FAILURE);
    std::string offString;
    unsigned int sizeV, sizeT, tmp;
    in >> offString >> sizeV >> sizeT >> tmp;
    vertices.resize (sizeV);
    triangles.resize (sizeT);
    for (unsigned int i = 0; i < sizeV; i++)
        in >> vertices[i].position;
    int s;
    for (unsigned int i = 0; i < sizeT; i++) {
        in >> s;
        for (unsigned int j = 0; j < 3; j++)
            in >> triangles[i].v[j];
    }
    in.close ();
}

void Mesh::recomputeNormals () {
    for (unsigned int i = 0; i < vertices.size (); i++)
        vertices[i].normal = Vec3 (0.0, 0.0, 0.0);
    for (unsigned int i = 0; i < triangles.size (); i++) {
        Vec3 e01 = vertices[triangles[i].v[1]].position -  vertices[triangles[i].v[0]].position;
        Vec3 e02 = vertices[triangles[i].v[2]].position -  vertices[triangles[i].v[0]].position;
        Vec3 n = Vec3::cross (e01, e02);
        n.normalize ();
        for (unsigned int j = 0; j < 3; j++)
            vertices[triangles[i].v[j]].normal += n;
    }
    for (unsigned int i = 0; i < vertices.size (); i++)
        vertices[i].normal.normalize ();
}

void Mesh::centerAndScaleToUnit () {
    Vec3 c(0,0,0);
    for  (unsigned int i = 0; i < vertices.size (); i++)
        c += vertices[i].position;
    c /= vertices.size ();
    float maxD = (vertices[0].position - c).length();
    for (unsigned int i = 0; i < vertices.size (); i++){
        float m = (vertices[i].position - c).length();
        if (m > maxD)
            maxD = m;
    }
    for  (unsigned int i = 0; i < vertices.size (); i++)
        vertices[i].position = (vertices[i].position - c) / maxD;
}

void Mesh::draw() const {
    if( triangles_array.size() == 0 ) return;
    GLfloat material_color[4] = {material.diffuse_material[0],
                                    material.diffuse_material[1],
                                    material.diffuse_material[2],
                                    1.0};

    GLfloat material_specular[4] = {material.specular_material[0],
                                    material.specular_material[1],
                                    material.specular_material[2],
                                    1.0};

    GLfloat material_ambient[4] = {material.ambient_material[0],
                                    material.ambient_material[1],
                                    material.ambient_material[2],
                                    1.0};

    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_color);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material_ambient);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material.shininess);

    glEnableClientState(GL_VERTEX_ARRAY) ;
    glEnableClientState (GL_NORMAL_ARRAY);
    glNormalPointer (GL_FLOAT, 3*sizeof (float), (GLvoid*)(normalsArray.data()));
    glVertexPointer (3, GL_FLOAT, 3*sizeof (float) , (GLvoid*)(positions_array.data()));
    glDrawElements(GL_TRIANGLES, triangles_array.size(), GL_UNSIGNED_INT, (GLvoid*)(triangles_array.data()));

}

void RayTraceMesh::draw() const {
    Mesh::draw();
    if(vertices.size() > 4){
        std::vector<AABB> aabbs;
        triangleTree->getAABBs(aabbs);

        for (const auto& box : aabbs) {
            box.draw();
        }
    }
}

RayIntersection RayTraceMesh::intersect( Ray const & ray ) const {
    RayTriangleIntersection closestIntersection;
    closestIntersection.t = FLT_MAX;
    closestIntersection.intersectionExists = false;
    
    auto possibleTriangleIdx = triangleTree->intersect(ray, *boundingBox);
    
    for(auto triangleIdx: possibleTriangleIdx){
        MeshTriangle meshTriangle = triangles[triangleIdx];

        Vec3 p0 = vertices[meshTriangle.v[0]].position * 1.0001;
        Vec3 p1 = vertices[meshTriangle.v[1]].position * 1.0001;
        Vec3 p2 = vertices[meshTriangle.v[2]].position * 1.0001;

        Triangle triangle = Triangle{p0, p1, p2};

        RayTriangleIntersection intersection = triangle.getIntersection(ray);
        if(intersection.intersectionExists && intersection.t < closestIntersection.t){
            closestIntersection = intersection;

            Vec3 n0 = vertices[meshTriangle.v[0]].normal;
            Vec3 n1 = vertices[meshTriangle.v[1]].normal;
            Vec3 n2 = vertices[meshTriangle.v[2]].normal;

            closestIntersection.normal = n0 * intersection.w0 + n1 * intersection.w1 + n2 * intersection.w2;
        }
    }

    RayIntersection result;
    result.intersectionExists = closestIntersection.intersectionExists;
    result.t = closestIntersection.t;
    result.intersection = closestIntersection.intersection;
    result.normal = closestIntersection.normal;
    result.normal.normalize();
    result.position = ray.origin() + ray.direction() * closestIntersection.t;
    result.material = material;
    result.u = closestIntersection.w0;
    result.v = closestIntersection.w1;


    return result;
}





void RayTraceMesh::buildTree(unsigned int nb_of_subdivide_tree){
    std::vector<Vec3> verticesPositions;
    Vec3 min, max;
    min = vertices[0].position;
    max = vertices[0].position;
    for (const auto& vertex : vertices) {
        verticesPositions.push_back(vertex.position);

        // Comparer chaque composante x, y, z du vecteur avec min et max
        for (int i = 0; i < 3; i++) { // 3 correspond à x, y, z
            if (vertex.position[i] < min[i]) {
                min[i] = vertex.position[i];
            }
            if (vertex.position[i] > max[i]) {
                max[i] = vertex.position[i];
            }
        }
    }

    Vec3 diag = max - min;
    diag.normalize();

    triangleTree = std::make_unique<BVH_Node>(BVH_Node::buildBVH(verticesPositions, triangles, nb_of_subdivide_tree, 0));
    boundingBox = std::make_unique<AABB>(triangleTree->boundingBox);
    // for(int idxT=0; idxT < triangles.size(); idxT ++){
    //     auto triangle = triangles[idxT];
    //     for(int i=0; i<3; i++){
    //         auto pos = vertices[triangle.v[i]].position;
    //         BVH_Node & currentTree = triangleTree.getCorrespondingTree(pos);
    //         bool hasAlreadyIdx = std::find(currentTree.trianglesIdx.begin(), currentTree.trianglesIdx.end(), idxT) != currentTree.trianglesIdx.end();
    //         if(!hasAlreadyIdx){
    //             currentTree.trianglesIdx.push_back(idxT);
    //         }
    //     }
    // }
}

AABB RayTraceMesh::getBoundingBox() const {
    return *boundingBox;
}
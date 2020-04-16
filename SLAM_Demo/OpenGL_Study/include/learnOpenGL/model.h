//
// Created by Robotics_qi on 2020/2/4.
//

#ifndef GUI_TOOL_MODEL_H
#define GUI_TOOL_MODEL_H

#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <learnOpenGL/mesh.h>
#include <learnOpenGL/shader.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>

using namespace std;

unsigned int TextureFromFile(const char* path, const string &directory, bool gamma = false);

class Model{
public:
    /* Model Data */

private:

};

#endif //GUI_TOOL_MODEL_H

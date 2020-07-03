//
// Created by Robotics_qi on 2020/7/2.
//

#ifndef LIV_OPENGL_GLSHADERCACHE_H
#define LIV_OPENGL_GLSHADERCACHE_H

#include <map>
#include <memory>
#include <string>

namespace LIV_OpenGL{
    //!@brief Shader缓存数据结构

    class GlShaderCache{
    public:
        //!@brief 缓冲对象的实例化
        static GlShaderCache& getInstance();

        //!@brief 通过文件获得缓冲区
        bool hasSource(const std::string& file_name);

        //!@brief 通过文件获取缓冲区
        std::string getSource(const std::string& filename);

        //!@brief 通过键值对生成缓冲区
        void insertSource(const std::string& filename, const std::string& source);

        //!@brief 从文件中读取和增加
        void insertSource(const std::string& filename);
    protected:
        GlShaderCache();
        GlShaderCache(const GlShaderCache&);
        GlShaderCache& operator=(const GlShaderCache&);

        std::string basename(const std::string& filename) const;

        std::map<std::string, std::string> sources_;
    };
}


#endif //LIV_OPENGL_GLSHADERCACHE_H

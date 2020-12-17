#pragma once

#include <map>

#include "PolyMesh_Base.h"
#include "PolyMesh.h"

namespace acamcad {
namespace polymesh {

struct IOOptions
{
    bool vert_have_normal;
    bool vert_have_texture;
    bool vert_have_color;

    bool face_has_color;
    bool face_has_texcoord;

    bool color_has_alpha;

    IOOptions() : vert_have_normal(false), vert_have_texture(false), vert_have_color(false),
        face_has_color(false), face_has_texcoord(false), color_has_alpha(false)
    {}

    void clear()
    {
        vert_have_normal = false;
        vert_have_texture = false;
        vert_have_color = false;

        face_has_color = false;
        face_has_texcoord = false;

        color_has_alpha = false;
    }
};

bool loadMesh(const std::string& _filename, PolyMesh *mesh);

bool loadMesh(const std::string& _filename, PolyMesh *mesh, IOOptions& opt);

bool writeMesh(const std::string& _filename, PolyMesh* mesh);

bool writeMesh(const std::string& _filename, PolyMesh* mesh, IOOptions& opt);


class OBJReader
{
public:
    OBJReader() {};
    ~OBJReader() {};

    bool read(const std::string& _filename, PolyMesh* mesh, IOOptions& opt);

    // Obj counts from 1 and not zero .. array counts from zero therefore -1
    bool loadMeshFromOBJ(std::istream& _in, PolyMesh* mesh, IOOptions& opt);

    bool can_u_read(const std::string& _filename) const;
    std::string get_description() const { return "Alias/Wavefront"; }
    std::string get_extensions()  const { return "obj"; }

private:
    class Material
    {
    public:

        Material() :Tr_(0), index_Kd_(0) { cleanup(); }

        void cleanup()
        {
            Kd_is_set_ = false;
            Ka_is_set_ = false;
            Ks_is_set_ = false;
            Tr_is_set_ = false;
            map_Kd_is_set_ = false;
        }

        bool is_valid(void) const
        {
            return Kd_is_set_ || Ka_is_set_ || Ks_is_set_ || Tr_is_set_ || map_Kd_is_set_;
        }

        bool has_Kd(void) { return Kd_is_set_; }
        bool has_Ka(void) { return Ka_is_set_; }
        bool has_Ks(void) { return Ks_is_set_; }
        bool has_Tr(void) { return Tr_is_set_; }
        bool has_map_Kd(void) { return map_Kd_is_set_; }

        void set_Kd(float r, float g, float b)
        {
            Kd_ = MRGBf(r, g, b); Kd_is_set_ = true;
        }

        void set_Ka(float r, float g, float b)
        {
            Ka_ = MRGBf(r, g, b); Ka_is_set_ = true;
        }

        void set_Ks(float r, float g, float b)
        {
            Ks_ = MRGBf(r, g, b); Ks_is_set_ = true;
        }

        void set_Tr(float t)
        {
            Tr_ = t;            Tr_is_set_ = true;
        }

        void set_map_Kd(const std::string& _name, int _index_Kd)
        {
            map_Kd_ = _name, index_Kd_ = _index_Kd; map_Kd_is_set_ = true;
        };

        const MRGBf& Kd(void) const { return Kd_; }
        const MRGBf& Ka(void) const { return Ka_; }
        const MRGBf& Ks(void) const { return Ks_; }
        float  Tr(void) const { return Tr_; }
        const std::string& map_Kd(void) { return map_Kd_; }
        const int& map_Kd_index(void) { return index_Kd_; }

    private:

        MRGBf Kd_;                          bool Kd_is_set_; // diffuse
        MRGBf Ka_;                          bool Ka_is_set_; // ambient
        MRGBf Ks_;                          bool Ks_is_set_; // specular
        float Tr_;                          bool Tr_is_set_; // transperency

        std::string map_Kd_; int index_Kd_; bool map_Kd_is_set_; // Texture

    };

    typedef std::map<std::string, Material> MaterialList;

    MaterialList materials_;

    bool read_material_obj(std::fstream& _in);

    std::string path_;
};


class OFFReader
{
public:
    OFFReader() {};
    ~OFFReader() {};

    bool read(const std::string& _filename, PolyMesh* mesh, IOOptions& opt);

    bool loadMeshFromOFF(std::istream& _in, PolyMesh* mesh, IOOptions& opt);

    bool can_u_read(const std::string& _filename) const;
    std::string get_description() const { return "Object File Format"; }
    std::string get_extensions()  const { return "off"; }
    std::string get_magic()       const { return "OFF"; }

private:
    bool can_u_read(std::istream& _is) const;
    mutable IOOptions options_;

    int getColorType(std::string& _line, bool _texCoordsAvailable) const;
};


class OBJWriter
{
public:

    OBJWriter() {};
    ~OBJWriter() {};

    std::string get_description() const { return "Alias/Wavefront"; }
    std::string get_extensions()  const { return "obj"; }

    bool can_u_write(const std::string& _filename) const;

    bool write(const std::string&, PolyMesh* mesh, IOOptions, std::streamsize _precision = 6) const;

    bool write(std::ostream&, PolyMesh* mesh, IOOptions, std::streamsize _precision = 6) const;

private:

    mutable std::string path_;
    mutable std::string objName_;

    mutable std::vector< MRGBf > material_;
    mutable std::vector< MRGBAf > materialA_;

    size_t getMaterial(MRGBf& color) const;

    size_t getMaterial(MRGBAf& color) const;

    bool writeMaterial(std::ostream& _out, PolyMesh* mesh, IOOptions) const;


};


class OFFWriter
{
public:

    OFFWriter() {};

    ~OFFWriter() {};

    std::string get_description() const { return "no description"; }
    std::string get_extensions()  const { return "off"; }

    bool can_u_write(const std::string& _filename) const;

    bool write(const std::string&, PolyMesh* mesh, IOOptions, std::streamsize _precision = 6) const;

    bool write(std::ostream&, PolyMesh* mesh, IOOptions, std::streamsize _precision = 6) const;

protected:
    bool write_ascii(std::ostream& _in, PolyMesh* mesh, IOOptions) const;
};

}//namespace polymesh
}//namespaec acamcad
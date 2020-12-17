#include "PolyMesh/IOManager.h"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace acamcad {
namespace polymesh {

#define LINE_LEN 4096

bool loadMesh(const std::string& _filename, PolyMesh* mesh)
{
	IOOptions opt;
	return loadMesh(_filename, mesh, opt);
}

bool loadMesh(const std::string& _filename, PolyMesh* mesh, IOOptions& opt)
{
    OBJReader* reader_obj = new OBJReader();
    OFFReader* reader_off = new OFFReader();

    if (reader_obj->can_u_read(_filename))
    {
        if (mesh == nullptr) {
            mesh = new PolyMesh();
        }
        else {
            mesh->clear();
        }   
        bool ok = reader_obj->read(_filename, mesh, opt);
        return ok;
    }
    if (reader_off->can_u_read(_filename))
    {
        if (mesh == nullptr) {
            mesh = new PolyMesh();
        }
        else {
            mesh->clear();
        }
        bool ok = reader_off->read(_filename, mesh, opt);
        return ok;
    }

    delete reader_obj;
    delete reader_off;

    std::ostringstream error_info;
    error_info << "[MeshReader] : Unrecognized format " << _filename << std::endl;
    Massage::Warning(error_info.str());
    return false;

}

bool writeMesh(const std::string& _filename, PolyMesh* mesh)
{
    IOOptions opt;
    return writeMesh(_filename, mesh, opt);
}

bool writeMesh(const std::string& _filename, PolyMesh* mesh, IOOptions& opt)
{
    OBJWriter* writer_obj = new OBJWriter();
    OFFWriter* writer_off = new OFFWriter();

    if (writer_obj->can_u_write(_filename))
    {
        bool ok = writer_obj->write(_filename, mesh, opt);
        return ok;
    }
    if (writer_off->can_u_write(_filename))
    {
        bool ok = writer_off->write(_filename, mesh, opt);
        return ok;
    }

    delete writer_obj;
    delete writer_off;

    std::ostringstream error_info;
    error_info << "[MeshReader] : Unrecognized format " << _filename << std::endl;
    Massage::Warning(error_info.str());
    return false;

}

//========================================================================
// Auxiliary function
//========================================================================

// Trim Both leading and trailing spaces    修建前后空格
void trimString(std::string& _string) {

    size_t start = _string.find_first_not_of(" \t\r\n");
    size_t end = _string.find_last_not_of(" \t\r\n");

    if ((std::string::npos == start) || (std::string::npos == end))
        _string = "";
    else
        _string = _string.substr(start, end - start + 1);
}

void remove_duplicated_vertices(std::vector<size_t>& _indices)
{
    std::vector<size_t>::iterator endIter = _indices.end();
    for (std::vector<size_t>::iterator iter = _indices.begin(); iter != endIter; ++iter)
        endIter = std::remove(iter + 1, endIter, *(iter));

    _indices.erase(endIter, _indices.end());
}

//========================================================================
// OBJReader 
//========================================================================

bool OBJReader::can_u_read(const std::string& _filename) const
{
    // get file extension
    std::string extension;
    std::string::size_type pos(_filename.rfind("."));

    if (pos != std::string::npos)
        extension = _filename.substr(pos + 1, _filename.length() - pos - 1);
    else
        extension = _filename; //check, if the whole filename defines the extension

    std::transform(extension.begin(), extension.end(), extension.begin(), tolower);

    // locate extension in extension string
    return (get_extensions().find(extension) != std::string::npos);
}

bool OBJReader::read(const std::string& _filename, PolyMesh* mesh, IOOptions& opt)
{
    std::fstream ifile(_filename.c_str(), std::ios_base::in);

    if (!ifile.is_open() || !ifile.good())
    {
        std::ostringstream error_info;
        error_info << "[OBJReader] : cannot not open file " << _filename << std::endl;
        Massage::Error(error_info.str());
        return false;
    }

    {
#if defined(WIN32)
        std::string::size_type dot_pos = _filename.find_last_of("\\/");
#else
        std::string::size_type dot_pos = _filename.rfind("/");
#endif
        path_ = (dot_pos == std::string::npos)
            ? "./"
            : std::string(_filename.substr(0, dot_pos + 1));
    }

    bool result = loadMeshFromOBJ(ifile, mesh, opt);

    ifile.close();
    return result;
}

bool OBJReader::loadMeshFromOBJ(std::istream& _in, PolyMesh* mesh, IOOptions& opt)
{
    if (!_in.good())
    {
        std::ostringstream error_info;
        error_info << "[OBJReader] : cannot not use stream " << std::endl;
        Massage::Error(error_info.str());
        return false;
    }

    // Options supplied by the user
    IOOptions user_options = opt;
    IOOptions file_options;


    double x, y, z, u, v, w;
    float r, g, b;

    std::vector<MVert*> vert_list;
    std::vector<MVector3> normal_list;
    std::vector<MRGBf> colors;
    std::vector<Texcoord> texcoords;

    std::string line;
    std::string keyWrd;

    std::stringstream stream;
    // pass 1: read vertices
    while (_in && !_in.eof())
    {
        std::getline(_in, line);
        if (_in.bad()) {
            std::ostringstream error_info;
            error_info << "[OBJReader] : Could not read file properly! " << std::endl;
            Massage::Error(error_info.str());
            return false;
        }

        trimString(line);

        // comment
        if (line.size() == 0 || line[0] == '#' || isspace(line[0])) {
            continue;
        }

        stream.str(line);   stream.clear(); //clear不是清除数据，而是清除附加信息，比如fail，bad这些流标志
        stream >> keyWrd;

        // vertex               "v"
        // texture coord        "vt"
        // color per vertex     "vc"
        // normal               "vn"
        if (keyWrd == "v")
        {
            stream >> x; stream >> y; stream >> z;

            if (!stream.fail())
            {
                vert_list.push_back(mesh->addVertex(x, y, z));

                stream >> r; stream >> g; stream >> b;

                if (!stream.fail())
                {
                    if (user_options.vert_have_color)
                    {
                        file_options.vert_have_color = true;
                        colors.push_back(MRGBf(r, g, b));
                    }
                }
            }
        }
        else if (keyWrd == "vt")
        {
            stream >> u; stream >> v;

            if (!stream.fail()) {
                if (user_options.vert_have_texture)
                {
                    texcoords.push_back(Texcoord(u,v));
                    file_options.vert_have_texture = true;

                    stream >> w;
                    if (!stream.fail())
                        texcoords[texcoords.size() - 1].setW(w);
                }
            }
            else {
                std::ostringstream error_info;
                error_info << "[OBJReader] : Only single 2D or 3D texture coordinate per vertex" << "allowed!" << std::endl;
                Massage::Error(error_info.str());
                return false;
            }
        }
        else if (keyWrd == "vc")
        {
            stream >> r; stream >> g; stream >> b;

            if (!stream.fail()) {
                if (user_options.vert_have_color)
                {
                    file_options.vert_have_color = true;
                    colors.push_back(MRGBf(r, g, b));
                }
            }
        }
        else if (keyWrd == "vn")
        {
            stream >> x; stream >> y; stream >> z;

            if (!stream.fail()) {
                if (user_options.vert_have_normal) {
                    normal_list.push_back(MVector3(x,y,z));
                    file_options.vert_have_normal = true;
                }
            }
        }
    }

    // reset stream for second pass
    _in.clear();
    _in.seekg(0, std::ios::beg);

    int nCurrentPositions = 0, nCurrentTexcoords = 0, nCurrentNormals = 0;
    std::vector<size_t>       face_vertid;
    std::vector<Texcoord>     face_texcoords;

    std::string               matname;
    std::stringstream         streamf, lineData, tmp;
    // pass 2: read faces
    while (_in && !_in.eof())
    {
        std::getline(_in, line);
        if (_in.bad()) 
        {
            std::ostringstream error_info;
            error_info << "[OBJReader] : Could not read file properly! " << std::endl;
            Massage::Error(error_info.str());
            return false;
        }

        trimString(line);

        // comment
        if (line.size() == 0 || line[0] == '#' || isspace(line[0])) {
            continue;
        }

        streamf.str(line);   streamf.clear();
        streamf >> keyWrd;

        // material file        "mtllib"
        // usemtl               "usemtl"
        // faces                "f"
        if (keyWrd == "mtllib")
        {
            std::string matFile;

            // Get the rest of the line, removing leading or trailing spaces
            // This will define the filename of the texture
            std::getline(streamf, matFile);
            trimString(matFile);

            matFile = path_ + matFile;
            std::fstream matStream(matFile.c_str(), std::ios_base::in);

            if (matStream) {

                if (!read_material_obj(matStream))
                {
                    std::ostringstream error_info;
                    error_info << "[OBJReader] : Could not read material file properly! " << std::endl;
                    Massage::Warning(error_info.str());
                }
                matStream.close();
            }
            else
            {
                std::ostringstream error_info;
                error_info << "[OBJReader] : Material file '" << matFile << "' not found!\n";
                Massage::Warning(error_info.str());
            }

            for (MaterialList::iterator material = materials_.begin(); material != materials_.end(); ++material)
            {
                // Save the texture information in a property
                if ((*material).second.has_map_Kd())
                    mesh->add_texture_information((*material).second.map_Kd_index(), (*material).second.map_Kd());
            }

        }
        else if (keyWrd == "usemtl")
        {
            streamf >> matname;
            if (materials_.find(matname) == materials_.end())
            {
                std::ostringstream error_info;
                error_info << "Material '" << matname << "' not defined in material file.\n";
                Massage::Warning(error_info.str());

                matname = "";
            }
        }
        // track current number of parsed vertex attributes, to allow for OBJs negative indices
        else if (keyWrd == "v")
        {
            ++nCurrentPositions;
        }
        else if (keyWrd == "vt")
        {
            ++nCurrentTexcoords;
        }
        else if (keyWrd == "vn")
        {
            ++nCurrentNormals;
        }
        else if (keyWrd == "f")
        {
            int component(0), nV(0);
            int value;

            face_vertid.clear();
            face_texcoords.clear();

            // read full line after detecting a face
            std::string faceLine;
            std::getline(streamf, faceLine);
            lineData.str(faceLine);
            lineData.clear();

            MPolyFace* fh;
            std::vector<size_t> faceVertices;

            // work on the line until nothing left to read
            while (!lineData.eof())
            {
                // read one block from the line ( vertex/texCoord/normal )
                std::string vertex;
                lineData >> vertex;

                do 
                {
                    //get the component (vertex/texCoord/normal)
                    size_t found = vertex.find("/");

                    // parts are seperated by '/' So if no '/' found its the last component
                    if (found != std::string::npos) 
                    {
                        // read the index value
                        tmp.str(vertex.substr(0, found));
                        tmp.clear();

                        // If we get an empty string this property is undefined in the file
                        if (vertex.substr(0, found).empty()) {
                            // Switch to next field
                            vertex = vertex.substr(found + 1);

                            // Now we are at the next component
                            ++component;

                            // Skip further processing of this component
                            continue;
                        }

                        // Read current value
                        tmp >> value;

                        // remove the read part from the string
                        vertex = vertex.substr(found + 1);

                    }
                    else 
                    {
                        // last component of the vertex, read it.
                        tmp.str(vertex);
                        tmp.clear();
                        tmp >> value;

                        // Clear vertex after finished reading the line
                        vertex = "";

                        // Nothing to read here ( garbage at end of line )
                        if (tmp.fail()) {
                            continue;
                        }
                    }

                    // store the component ( each component is referenced by the index here! )

                    // Calculation of index : -1 is the last vertex in the list
                    // As obj counts from 1 and not zero add +1
                    switch (component)
                    {
                    case 0: // vertex
                        if (value < 0) {
                            value = nCurrentTexcoords + value + 1;
                        }
                        face_vertid.push_back(value - 1);
                        faceVertices.push_back(value - 1);
                        if (file_options.vert_have_color) 
                        {
                            if ((size_t)(value - 1) < colors.size()) {
                                mesh->vert(value - 1)->setColor(colors[value - 1]);
                            }
                            else {
                                std::ostringstream error_info;
                                error_info << "[OBJReader] : Error setting vertex color " << std::endl;
                                Massage::Warning(error_info.str());
                            }
                        }
                        break;

                    case 1: // texture coord
                        if (value < 0) {
                            value = nCurrentNormals + value + 1;
                        }
                        assert(!face_vertid.empty());

                        if (file_options.vert_have_texture) 
                        {
                            if (!texcoords.empty() && (unsigned int)(value - 1) < texcoords.size()) {
                                mesh->vert(value - 1)->setTexture(texcoords[value - 1]);
                            }
                            else {
                                std::ostringstream error_info;
                                error_info << "[OBJReader] : Error setting Texture coordinates " << std::endl;
                                Massage::Warning(error_info.str());
                            }
                        }
                        break;

                    case 2: // normal
                        if (value < 0) {
                            value = nCurrentNormals + value + 1;
                        }

                        if (file_options.vert_have_normal) 
                        {
                            assert(!face_vertid.empty());
                            if ((unsigned int)(value - 1) < normal_list.size()) {
                                mesh->vert(value - 1)->setNormal(normal_list[value - 1]);
                            }
                            else {
                                std::ostringstream error_info;
                                error_info << "[OBJReader] : Error setting vertex normal " << std::endl;
                                Massage::Warning(error_info.str());
                            }
                        }
                        break;
                    }

                    // Prepare for reading next component
                    ++component;

                    // Read until line does not contain any other info
                } while (!vertex.empty());

                component = 0;
                nV++;

            }

            remove_duplicated_vertices(faceVertices);

            //A minimum of three vertices are required.
            size_t n_faces = mesh->numPolygons();
            if (faceVertices.size() > 2)
                fh = mesh->addPolyFace(faceVertices);

            if (!matname.empty())
            {
                MPolyFace* newface = mesh->polyface(n_faces);

                Material& mat = materials_[matname];

                if (mat.has_Kd()) {
                    MRGBf fc = mat.Kd();

                    if (user_options.face_has_color) 
                    {
                        newface->setColor(fc);
                        file_options.face_has_color = true;
                    }
                }

                // Set the texture index in the face index property
                if (mat.has_map_Kd()) {

                    if (user_options.face_has_texcoord) 
                    {
                        newface->setMaterialIndex(mat.map_Kd_index());
                        file_options.face_has_texcoord = true;
                    }
                }
                else 
                {
                    // If we don't have the info, set it to no texture
                    if (user_options.face_has_texcoord) 
                    {
                        newface->setMaterialIndex(0);
                    }
                }
            }
            else 
            {
                MPolyFace* newface = mesh->polyface(n_faces);

                // Set the texture index to zero as we don't have any information
                if (user_options.face_has_texcoord)
                {
                    newface->setMaterialIndex(0);
                }
            }

        }
    }

    // If we do not have any faces,
    // assume this is a point cloud and read the normals and colors directly
    if (mesh->numPolygons() == 0)
    {
        size_t vertnum = vert_list.size();
        // add normal per vertex
        if (normal_list.size() == mesh->numVertices()) 
        {
            if (user_options.vert_have_normal && file_options.vert_have_normal) 
            {
                for (size_t i = 0; i < vertnum; i++)
                    vert_list[i]->setNormal(normal_list[i]);
            }
        }

        // add color per vertex
        if (colors.size() >= mesh->numVertices())
        {
            if (user_options.vert_have_color && file_options.vert_have_color)
            {
                for (size_t i = 0; i < vertnum; i++)
                    vert_list[i]->setColor(colors[i]);
            }
        }
    }

    // Return, what we actually read
    opt = file_options;

    return true;
}

bool OBJReader::read_material_obj(std::fstream& _in)
{
    std::string line;
    std::string keyWrd;
    std::string textureName;

    std::stringstream  stream;

    std::string key;
    Material    mat;
    float       f1, f2, f3;
    bool        indef = false;
    int         textureId = 1;


    materials_.clear();
    mat.cleanup();

    while (_in && !_in.eof())
    {
        std::getline(_in, line);
        if (_in.bad()) 
        {
            std::ostringstream error_info;
            error_info << "[OBJReader Material] : Could not read file properly! " << std::endl;
            Massage::Warning(error_info.str());
            return false;
        }

        if (line.empty())
            continue;

        stream.str(line);   stream.clear();
        stream >> keyWrd;

        if ((isspace(line[0]) && line[0] != '\t') || line[0] == '#')
        {
            if (indef && !key.empty() && mat.is_valid())
            {
                materials_[key] = mat;
                mat.cleanup();
            }
        }

        else if (keyWrd == "newmtl") // begin new material definition
        {
            stream >> key;
            indef = true;
        }

        else if (keyWrd == "Kd") // diffuse color
        {
            stream >> f1; stream >> f2; stream >> f3;

            if (!stream.fail())
                mat.set_Kd(f1, f2, f3);
        }

        else if (keyWrd == "Ka") // ambient color
        {
            stream >> f1; stream >> f2; stream >> f3;

            if (!stream.fail())
                mat.set_Ka(f1, f2, f3);
        }

        else if (keyWrd == "Ks") // specular color
        {
            stream >> f1; stream >> f2; stream >> f3;

            if (!stream.fail())
                mat.set_Ks(f1, f2, f3);
        }
        else if (keyWrd == "map_Kd") {
            // Get the rest of the line, removing leading or trailing spaces
            // This will define the filename of the texture
            std::getline(stream, textureName);
            trimString(textureName);
            if (!textureName.empty())
                mat.set_map_Kd(textureName, textureId++);
        }
        else if (keyWrd == "Tr") // transparency value
        {
            stream >> f1;

            if (!stream.fail())
                mat.set_Tr(f1);
        }
        else if (keyWrd == "d") // transparency value
        {
            stream >> f1;

            if (!stream.fail())
                mat.set_Tr(f1);
        }

        if (_in && indef && mat.is_valid() && !key.empty())
            materials_[key] = mat;
    }
    return true;
}

//========================================================================
// OFFReader 
//========================================================================

bool OFFReader::can_u_read(const std::string& _filename) const
{
    // get file extension
    std::string extension;
    std::string::size_type pos(_filename.rfind("."));

    if (pos != std::string::npos)
        extension = _filename.substr(pos + 1, _filename.length() - pos - 1);
    else
        extension = _filename; //check, if the whole filename defines the extension

    std::transform(extension.begin(), extension.end(), extension.begin(), tolower);

    // locate extension in extension string
    if ((get_extensions().find(extension) != std::string::npos))
    {
        std::ifstream ifs(_filename.c_str());
        if (ifs.is_open() && can_u_read(ifs))
        {
            ifs.close();
            return true;
        }
    }
    return false;
}

bool OFFReader::read(const std::string& _filename, PolyMesh* mesh, IOOptions& opt)
{
    std::ifstream ifile(_filename.c_str(), std::ios::in);

    if (!ifile.is_open() || !ifile.good())
    {
        std::ostringstream error_info;
        error_info << "[OFFReader] : cannot not open file " << _filename << std::endl;
        Massage::Error(error_info.str());
        return false;
    }

    assert(ifile);

    bool result = loadMeshFromOFF(ifile, mesh, opt);

    ifile.close();
    return result;
}

bool OFFReader::loadMeshFromOFF(std::istream& _in, PolyMesh* mesh, IOOptions& opt)
{
    if (!_in.good())
    {
        std::ostringstream error_info;
        error_info << "[OFFReader] : cannot not use stream " << std::endl;
        Massage::Error(error_info.str());
        return false;
    }

    IOOptions user_options = opt;

    // build options to be returned
    opt.clear();

    if (options_.vert_have_normal && user_options.vert_have_normal)     opt.vert_have_normal;
    if (options_.vert_have_texture && user_options.vert_have_texture)   opt.vert_have_texture;
    if (options_.vert_have_color && user_options.vert_have_color)       opt.vert_have_color;
    if (options_.face_has_color && user_options.face_has_color)     opt.face_has_color;


    unsigned int              i, j, idx;
    unsigned int              nV, nF, dummy;
    double vx, vy, vz, nx, ny, nz;
    int c0, c1, c2, c3;
    float cf0, cf1, cf2, cf3;
    float t0, t1;

    std::stringstream         stream;
    std::string               trash;

    // read header line
    std::string header;
    std::getline(_in, header);

    // + #Vertice, #Faces, #Edges
    _in >> nV;
    _in >> nF;
    _in >> dummy;

    mesh->reserveMemory(nV, nF);

    MVert* pvert;
    // read vertices: coord [hcoord] [normal] [color] [texcoord]
    for (i = 0; i < nV && !_in.eof(); ++i)
    {
        // Always read VERTEX
        _in >> vx; _in >> vy; _in >> vz;

        pvert = mesh->addVertex(vx, vy, vz);

        //perhaps read NORMAL
        if (options_.vert_have_normal) 
        {
            _in >> nx; _in >> ny; _in >> nz;

            if (user_options.vert_have_normal)
                pvert->setNormal(nx, ny, nz);
        }

        //take the rest of the line and check how colors are defined
        std::string line;
        std::getline(_in, line);

        int colorType = getColorType(line, options_.vert_have_texture);

        stream.str(line);
        stream.clear();

        //perhaps read COLOR
        if (options_.vert_have_color) 
        {
            switch (colorType) 
            {
            case 0: break; //no color
            case 1: stream >> trash; break; //one int (isn't handled atm)
            case 2: stream >> trash; stream >> trash; break; //corrupt format (ignore)
            // rgb int
            case 3: stream >> c0;  stream >> c1;  stream >> c2;
                if (user_options.vert_have_color)
                {
                    MRGB c(c0, c1, c2); MRGBf cf = RGB_to_RGBF(c);
                    pvert->setColor(cf);
                }
                break;
                // rgba int
            case 4: stream >> c0;  stream >> c1;  stream >> c2; stream >> c3;
                if (user_options.vert_have_color)
                {
                    MRGBA c(c0, c1, c2, c3); MRGBAf cf = RGBA_to_RGBAF(c);
                    pvert->setColor(cf);
                }
                break;
                // rgb floats
            case 5: stream >> cf0;  stream >> cf1;  stream >> cf2;
                if (user_options.vert_have_color) {
                    pvert->setColor(cf0, cf1, cf2);
                }
                break;
                // rgba floats
            case 6: stream >> cf0;  stream >> cf1;  stream >> cf2; stream >> cf3;
                if (user_options.vert_have_color) {
                    pvert->setColor(cf0, cf1, cf2, cf3);
                }
                break;

            default:
                std::cerr << "Error in file format (colorType = " << colorType << ")\n";
                break;
            }
        }
        //perhaps read TEXTURE COORDs
        if (options_.vert_have_texture) {
            stream >> t0; stream >> t1;
            if (user_options.vert_have_texture)
                pvert->setTexture(t0, t1);
        }
    }

    std::vector<size_t> fvert_id;
    MPolyFace* pface;

    // faces
    // #N <v1> <v2> .. <v(n-1)> [color spec]
    for (i = 0; i < nF; ++i)
    {
        // nV = number of Vertices for current face
        _in >> nV;

        if (nV == 3)
        {
            fvert_id.resize(3);
            _in >> fvert_id[0];
            _in >> fvert_id[1];
            _in >> fvert_id[2];
        }
        else
        {
            fvert_id.clear();
            for (j = 0; j < nV; ++j)
            {
                _in >> idx;
                fvert_id.push_back(idx);
            }
        }

        pface = mesh->addPolyFace(fvert_id);

        //perhaps read face COLOR
        if (options_.face_has_color) {

            //take the rest of the line and check how colors are defined
            std::string line;
            std::getline(_in, line);

            int colorType = getColorType(line, false);

            stream.str(line);
            stream.clear();

            switch (colorType) {
            case 0: break; //no color
            case 1: stream >> trash; break; //one int (isn't handled atm)
            case 2: stream >> trash; stream >> trash; break; //corrupt format (ignore)
            // rgb int
            case 3: stream >> c0;  stream >> c1;  stream >> c2;
                if (user_options.face_has_color) 
                {
                    MRGB c(c0, c1, c2); MRGBf cf = RGB_to_RGBF(c);
                    pface->setColor(cf);
                }
                break;
                // rgba int
            case 4: stream >> c0;  stream >> c1;  stream >> c2; stream >> c3;
                if (user_options.face_has_color) 
                {
                    MRGBA c(c0, c1, c2, c3); MRGBAf cf = RGBA_to_RGBAF(c);
                    pface->setColor(cf);
                }
                break;
                // rgb floats
            case 5: stream >> cf0;  stream >> cf1;  stream >> cf2;
                if (user_options.face_has_color) {
                    pface->setColor(cf0, cf1, cf2);
                }
                break;
                // rgba floats
            case 6: stream >> cf0;  stream >> cf1;  stream >> cf2; stream >> cf3;
                if (user_options.face_has_color) {
                    pface->setColor(cf0, cf1, cf2, cf3);
                }
                break;

            default:
                std::cerr << "Error in file format (colorType = " << colorType << ")\n";
                break;
            }
        }
    }

    // File was successfully parsed.
    return true;

}

bool OFFReader::can_u_read(std::istream& _is) const
{
    options_.clear();

    // read 1st line
    char line[LINE_LEN], * p;
    _is.getline(line, LINE_LEN);
    p = line;

    std::streamsize remainingChars = _is.gcount();

    bool vertexDimensionTooHigh = false;

    // check header: [ST][C][N][4][n]OFF BINARY

    if ((remainingChars > 1) && (p[0] == 'S' && p[1] == 'T'))
    {
        options_.vert_have_texture; p += 2; remainingChars -= 2;
    }

    if ((remainingChars > 0) && (p[0] == 'C'))
    {
        options_.vert_have_color;
        options_.face_has_color; ++p; --remainingChars;
    }

    if ((remainingChars > 0) && (p[0] == 'N'))
    {
        options_.vert_have_normal; ++p; --remainingChars;
    }

    if ((remainingChars > 0) && (p[0] == '4'))
    {
        vertexDimensionTooHigh = true; ++p; --remainingChars;
    }

    if ((remainingChars > 0) && (p[0] == 'n'))
    {
        vertexDimensionTooHigh = true; ++p; --remainingChars;
    }

    if ((remainingChars < 3) || (!(p[0] == 'O' && p[1] == 'F' && p[2] == 'F')))
        return false;

    p += 4;

    // Detect possible garbage and make sure, we don't have an underflow
    if (remainingChars >= 4)
        remainingChars -= 4;
    else
        remainingChars = 0;

    //if ((remainingChars >= 6) && (strncmp(p, "BINARY", 6) == 0))
    //    options_ += Options::Binary;

    // vertex Dimensions != 3 are currently not supported
    if (vertexDimensionTooHigh)
        return false;

    return true;
}

int OFFReader::getColorType(std::string& _line, bool _texCoordsAvailable) const
{
    /*
      0 : no Color
      1 : one int (e.g colormap index)
      2 : two items (error!)
      3 : 3 ints
      4 : 3 ints
      5 : 3 floats
      6 : 4 floats
    */
    // Check if we have any additional information here
    if (_line.size() < 1)
        return 0;

    //first remove spaces at start/end of the line
    while (_line.size() > 0 && isspace(_line[0]))
        _line = _line.substr(1);
    while (_line.size() > 0 && isspace(_line[_line.length() - 1]))
        _line = _line.substr(0, _line.length() - 1);

    //count the remaining items in the line
    size_t found;
    int count = 0;

    found = _line.find_first_of(" ");
    while (found != std::string::npos) {
        count++;
        found = _line.find_first_of(" ", found + 1);
    }

    if (!_line.empty()) count++;

    if (_texCoordsAvailable) count -= 2;

    if (count == 3 || count == 4) {
        //get first item
        found = _line.find(" ");
        std::string c1 = _line.substr(0, found);

        if (c1.find(".") != std::string::npos) {
            if (count == 3)
                count = 5;
            else
                count = 6;
        }
    }
    return count;
}

//========================================================================
// OBJWriter 
//========================================================================

bool OBJWriter::write(const std::string& _filename, PolyMesh* mesh, IOOptions _opt, std::streamsize _precision) const
{
    std::fstream out(_filename.c_str(), std::ios_base::out);

    if (!out)
    {
        std::ostringstream error_info;
        error_info << "[OBJWriter] : cannot not open file " << _filename << std::endl;
        Massage::Error(error_info.str());
        return false;
    }

    // Set precision on output stream. The default is set via IOManager and passed through to all writers.
    out.precision(_precision);

    // Set fixed output to avoid problems with programs not reading scientific notation correctly
    out << std::fixed;

    {
#if defined(WIN32)
        std::string::size_type dotposition = _filename.find_last_of("\\/");
#else
        std::string::size_type dotposition = _filename.rfind("/");
#endif

        if (dotposition == std::string::npos) {
            path_ = "./";
            objName_ = _filename;
        }
        else {
            path_ = _filename.substr(0, dotposition + 1);
            objName_ = _filename.substr(dotposition + 1);
        }

        //remove the file extension
        dotposition = objName_.find_last_of(".");

        if (dotposition != std::string::npos)
            objName_ = objName_.substr(0, dotposition);
    }

    bool result = write(out, mesh, _opt, _precision);

    out.close();
    return result;
}

bool OBJWriter::write(std::ostream& _out, PolyMesh* mesh, IOOptions _opt, std::streamsize _precision) const
{
    unsigned int idx;
    MPoint3 v; MVector3 n;
    Texcoord t;
    MVert* vh;
    std::vector<MVert*> vhandles;
    MRGBAf caf; MRGBf cf;
    bool useMatrial = false;

    {
        std::ostringstream error_info;
        error_info << "[OBJWriter] : Begin to write file " << std::endl;
        Massage::Info(error_info.str());
    }

    _out.precision(_precision);

    // check for unsupported writer features
    if (_opt.vert_have_color) 
    {
        std::ostringstream error_info;
        error_info << "[OBJWriter] : VertexColor not supported by OBJ Writer" << std::endl;
        Massage::Error(error_info.str());
        return false;
    }

    //create material file if needed
    if (_opt.face_has_color) 
    {
        std::string matFile = path_ + objName_ + ".mat";
        std::fstream matStream(matFile.c_str(), std::ios_base::out);

        if (!matStream)
        {
            std::ostringstream error_info;
            error_info << "[OBJWriter] : cannot write material file " << matFile << std::endl;
            Massage::Error(error_info.str());
            return false;
        }
        else 
        {
            useMatrial = writeMaterial(matStream, mesh, _opt);
            matStream.close();
        }
    }

    // header
    _out << "# " << mesh->numVertices() << " vertices, ";
    _out << mesh->numPolygons() << " faces" << '\n';

    // material file
    if (useMatrial && _opt.face_has_color)
        _out << "mtllib " << objName_ << ".mat" << '\n';

    std::map<Texcoord, int> texMap;
    //collect Texturevertices from halfedges
    if (_opt.face_has_texcoord)
    {
        std::vector<Texcoord> texCoords;
        //add all texCoords to map

        size_t num = mesh->getFaceTexcoords(texCoords);
        for (size_t i = 0; i < num; ++i)
        {
            texMap[texCoords[i]] = i;
        }
    }

    //collect Texture coordinates from vertices
    if (_opt.vert_have_texture)
    {
        for (size_t i = 0, nV = mesh->numVertices(); i < nV; ++i)
        {
            vh = mesh->vert(i);
            t = vh->getTexture();

            texMap[t] = static_cast<int>(i);
        }
    }

    // assign each texcoord in the map its id
    // and write the vt entries
    if (_opt.vert_have_texture || _opt.face_has_texcoord)
    {
        int texCount = 0;
        for (std::map<Texcoord, int>::iterator it = texMap.begin(); it != texMap.end(); ++it)
        {
            const Texcoord& t = it->first;
            _out << "vt " << t[0] << " " << t[1] << '\n';
            it->second = ++texCount;
        }
    }

    // vertex data (point, normals, texcoords)
    for (size_t i = 0, nV = mesh->numVertices(); i < nV; ++i)
    {
        vh = mesh->vert(i);
        v = vh->position();
        n = vh->normal();

        _out << "v " << v[0] << " " << v[1] << " " << v[2] << '\n';

        if (_opt.vert_have_normal)
            _out << "vn " << n[0] << " " << n[1] << " " << n[2] << '\n';
    }

    size_t lastMat = std::numeric_limits<std::size_t>::max();

    // we do not want to write seperators if we only write vertex indices
    bool onlyVertices = !_opt.vert_have_texture && !_opt.vert_have_normal && !_opt.face_has_texcoord;

    // faces (indices starting at 1 not 0)
    for (size_t i = 0, nF = mesh->numPolygons(); i < nF; ++i)
    {
        if (useMatrial && _opt.face_has_color) {
            size_t material = std::numeric_limits<std::size_t>::max();

            MPolyFace* pface = mesh->polyface(i);
            //color with alpha
            if (_opt.color_has_alpha) {
                caf = pface->getColor_with_alpha();
                material = getMaterial(caf);
            }
            else {
                //and without alpha
                cf = pface->getColor();
                material = getMaterial(cf);
            }

            // if we are ina a new material block, specify in the file which material to use
            if (lastMat != material) {
                _out << "usemtl mat" << material << '\n';
                lastMat = material;
            }
        }

        _out << "f";

        MPolyFace* face = mesh->polyface(i);
        vhandles = mesh->polygonVertices(face);

        for (size_t j = 0; j < vhandles.size(); ++j)
        {

            // Write vertex index
            idx = vhandles[j]->index() +1;
            _out << " " << idx;

            if (!onlyVertices) {
                // write separator
                _out << "/";

                //write texCoords index from halfedge
                if (_opt.face_has_texcoord)
                {
                    MHalfedge* he = mesh->faceHalfEdgeFromVert(face, vhandles[j]);
                    _out << texMap[he->getTexture()];
                }
                else
                {
                    // write vertex texture coordinate index
                    if (_opt.vert_have_texture)
                    {
                        t = vhandles[j]->getTexture();
                        _out << texMap[t];
                    }
                }

                // write vertex normal index
                if (_opt.vert_have_normal) {
                    // write separator
                    _out << "/";
                    _out << idx;
                }
            }
        }

        _out << '\n';
    }

    material_.clear();
    materialA_.clear();

    return true;
}

size_t OBJWriter::getMaterial(MRGBf& color) const
{
    for (size_t i = 0; i < material_.size(); i++)
        if (equality(color, material_[i]))
            return i;

    //not found add new material
    material_.push_back(color);
    return material_.size() - 1;
}

size_t OBJWriter::getMaterial(MRGBAf& color) const
{
    for (size_t i = 0; i < materialA_.size(); i++)
        if (equality(color, materialA_[i]))
            return i;

    //not found add new material
    materialA_.push_back(color);
    return materialA_.size() - 1;
}

bool OBJWriter::writeMaterial(std::ostream& _out, PolyMesh* mesh, IOOptions _opt) const
{
    MRGBf cf;
    MRGBAf caf;

    material_.clear();
    materialA_.clear();

    //iterate over faces
    for (size_t i = 0, nF = mesh->numPolygons(); i < nF; ++i)
    {
        MPolyFace* face = mesh->polyface(i);
        //color with alpha
        if (_opt.color_has_alpha) {
            caf = face->getColor_with_alpha();
            getMaterial(caf);
        }
        else {  //and without alpha
            cf = face->getColor();
            getMaterial(cf);
        }
    }

    //write the materials
    if (_opt.color_has_alpha)
        for (size_t i = 0; i < materialA_.size(); i++) {
            _out << "newmtl " << "mat" << i << '\n';
            _out << "Ka 0.5000 0.5000 0.5000" << '\n';
            _out << "Kd " << materialA_[i].r << ' ' << materialA_[i].g << ' ' << materialA_[i].b << '\n';
            _out << "Tr " << materialA_[i].a << '\n';
            _out << "illum 1" << '\n';
        }
    else
        for (size_t i = 0; i < material_.size(); i++) {
            _out << "newmtl " << "mat" << i << '\n';
            _out << "Ka 0.5000 0.5000 0.5000" << '\n';
            _out << "Kd " << material_[i].r << ' ' << material_[i].g << ' ' << material_[i].b << '\n';
            _out << "illum 1" << '\n';
        }

    return true;
}

bool OBJWriter::can_u_write(const std::string& _filename) const
{
    // get file extension
    std::string extension;
    std::string::size_type pos(_filename.rfind("."));

    if (pos != std::string::npos)
        extension = _filename.substr(pos + 1, _filename.length() - pos - 1);
    else
        extension = _filename; //check, if the whole filename defines the extension

    std::transform(extension.begin(), extension.end(),
        extension.begin(), tolower);

    // locate extension in extension string
    return (get_extensions().find(extension) != std::string::npos);
}

//========================================================================
// OFFWriter 
//========================================================================

bool OFFWriter::write(const std::string& _filename, PolyMesh* mesh, IOOptions _opt, std::streamsize _precision) const
{
    std::ofstream out(_filename.c_str(), std::ios_base::out);

    return write(out, mesh, _opt, _precision);
}

bool OFFWriter:: write(std::ostream& _os, PolyMesh* mesh, IOOptions _opt, std::streamsize _precision) const
{
    if (!_os.good())
    {
        std::ostringstream error_info;
        error_info << "[OFFWriter] : cannot write to stream " << std::endl;
        Massage::Error(error_info.str());

        return false;
    }

    // write header line
    if (_opt.vert_have_texture) _os << "ST";
    if (_opt.vert_have_color || _opt.face_has_color)    _os << "C";
    if (_opt.vert_have_normal)  _os << "N";
    _os << "OFF";
    _os << "\n";

    // write to file
    bool result = write_ascii(_os, mesh, _opt);

    // return result
    return result;
}

bool OFFWriter::write_ascii(std::ostream& _out, PolyMesh* mesh, IOOptions _opt) const
{
    size_t nV, nF;
    MPoint3 v; MVector3 n;

    MRGBf cf;
    MRGBAf caf;

    Texcoord t;

    MVert* pvert;
    std::vector<MVert*> vhandles;


    // #vertices, #faces
    _out << mesh->numVertices() << " ";
    _out << mesh->numPolygons() << " ";
    _out << 0 << "\n";

    _out << std::fixed;


    // vertex data (point, normals, colors, texcoords)
    for (size_t i = 0, nV = mesh->numVertices(); i < nV; ++i)
    {
        pvert = mesh->vert(i);
        v = pvert->position();

        //Vertex
        _out << v[0] << " " << v[1] << " " << v[2];

        // VertexNormal
        if (_opt.vert_have_normal) {
            n = pvert->normal();
            _out << " " << n[0] << " " << n[1] << " " << n[2];
        }

        // VertexColor
        if (_opt.vert_have_color) 
        {
            if (_opt.color_has_alpha) {
                caf = pvert->getColor_with_alpha();
                _out << " " << caf.r << " " << caf.g << " " << caf.b << " " << caf.a;
            }
            else {
                cf = pvert->getColor();
                _out << " " << cf.r << " " << cf.g << " " << cf.b;
            }

        }

        // TexCoord
        if (_opt.vert_have_texture) {
            t = pvert->getTexture();
            _out << " " << t[0] << " " << t[1];
        }

        _out << '\n';

    }

    // faces (indices starting at 0)
    if (mesh->isTriangleMesh())
    {
        for (size_t i = 0, nF = mesh->numPolygons(); i < nF; ++i)
        {
            MPolyFace* pface = mesh->polyface(i);
            vhandles = mesh->polygonVertices(pface);

            _out << 3 << " ";
            _out << vhandles[0]->index() << " ";
            _out << vhandles[1]->index() << " ";
            _out << vhandles[2]->index();

            //face color
            if (_opt.face_has_color) 
            {
                if (_opt.color_has_alpha) {
                    caf = pface->getColor_with_alpha();
                    _out << " " << caf.r << " " << caf.g << " " << caf.b << " " << caf.a;
                }
                else {
                    cf = pface->getColor();
                    _out << " " << cf.r << " " << cf.g << " " << cf.b;
                }
            }
            _out << '\n';
        }
    }
    else
    {
        for (size_t i = 0, nF = mesh->numPolygons(); i < nF; ++i)
        {
            MPolyFace* pface = mesh->polyface(i);
            vhandles = mesh->polygonVertices(pface);
            nV = vhandles.size();

            _out << nV << " ";
            for (size_t j = 0; j < vhandles.size(); ++j)
                _out << vhandles[j]->index() << " ";

            //face color
            if (_opt.face_has_color) 
            {
                if (_opt.color_has_alpha) {
                    caf = pface->getColor_with_alpha();
                    _out << " " << caf.r << " " << caf.g << " " << caf.b << " " << caf.a;
                }
                else {
                    cf = pface->getColor();
                    _out << " " << cf.r << " " << cf.g << " " << cf.b;
                }
            }

            _out << '\n';
        }
    }


    return true;
}

bool OFFWriter:: can_u_write(const std::string& _filename) const
{
    // get file extension
    std::string extension;
    std::string::size_type pos(_filename.rfind("."));

    if (pos != std::string::npos)
        extension = _filename.substr(pos + 1, _filename.length() - pos - 1);
    else
        extension = _filename; //check, if the whole filename defines the extension

    std::transform(extension.begin(), extension.end(),
        extension.begin(), tolower);

    // locate extension in extension string
    return (get_extensions().find(extension) != std::string::npos);
}

}//namespace polymesh
}//namespaec acamcad
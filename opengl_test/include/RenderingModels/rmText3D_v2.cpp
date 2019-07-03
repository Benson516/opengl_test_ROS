#include "rmText3D_v2.h"

// Maximum texture width
#define MAXWIDTH 1024
#define TOTAL_CHAR 128




// Atlas
//--------------------------------------//
struct atlas {
	GLuint TextureID;		// texture object
    int font_size;

	unsigned int w;			// width of texture in pixels
	unsigned int h;			// height of texture in pixels

    /*
	struct {
		float ax;	// advance.x
		float ay;	// advance.y

		float bw;	// bitmap.width;
		float bh;	// bitmap.height;

		float bl;	// bitmap_left;
		float bt;	// bitmap_top;

		float tx;	// x offset of glyph in texture coordinates
		float ty;	// y offset of glyph in texture coordinates
	} _ch[TOTAL_CHAR];		// character information
    */
    struct {
		int ax;	// advance.x
		int ay;	// advance.y

		int bw;	// bitmap.width;
		int bh;	// bitmap.height;

		int bl;	// bitmap_left;
		int bt;	// bitmap_top;

		float tx;	// x offset of glyph in texture coordinates
		float ty;	// y offset of glyph in texture coordinates
	} _ch[TOTAL_CHAR];		// character information

    atlas(int font_size_in){
        // FreeType
        //-----------------------------------------//
        FT_Library ft;
        // All functions return a value different than 0 whenever an error occurred
        if (FT_Init_FreeType(&ft))
            std::cout << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
        // Load font as face
        FT_Face face;
        // if (FT_New_Face(ft, "fonts/arial.ttf", 0, &face))
        if (FT_New_Face(ft, "/usr/share/fonts/truetype/freefont/FreeSans.ttf", 0, &face))
        // if (FT_New_Face(ft, "FreeSans.ttf", 0, &face))
            std::cout << "ERROR::FREETYPE: Failed to load font" << std::endl;
        //-----------------------------------------//
        Init(face, font_size_in);
        // Destroy FreeType once we're finished
        //-----------------------------------------//
        FT_Done_Face(face);
        FT_Done_FreeType(ft);
        //-----------------------------------------//

    }
    atlas(FT_Face face, int font_size_in){
        Init(face, font_size_in);
    }
    void Init(FT_Face face, int font_size_in) {
        FT_Set_Pixel_Sizes(face, 0, font_size_in);
        font_size = font_size_in;
        FT_GlyphSlot g = face->glyph;

        unsigned int roww = 0;
        unsigned int rowh = 0;
        w = 0;
        h = 0;

        memset(_ch, 0, sizeof _ch);

        /* Find minimum size for a texture holding all visible ASCII characters */
        for (int i = 0; i < TOTAL_CHAR; i++) {
        	if (FT_Load_Char(face, i, FT_LOAD_RENDER)) {
        		fprintf(stderr, "Loading character %c failed!\n", i);
        		continue;
        	}
        	if (roww + g->bitmap.width + 1 >= MAXWIDTH) {
        		w = std::max(w, roww);
        		h += rowh + 1;
        		roww = 0;
        		rowh = 0;
        	}
        	roww += g->bitmap.width + 1;
        	rowh = std::max(rowh, g->bitmap.rows);
        }

        w = std::max(w, roww);
        h += rowh + 1;

        /* Create a texture that will be used to hold all ASCII glyphs */
        glActiveTexture(GL_TEXTURE0);
        glGenTextures(1, &TextureID);
        glBindTexture(GL_TEXTURE_2D, TextureID);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, w, h, 0, GL_ALPHA, GL_UNSIGNED_BYTE, 0);
        /* We require 1 byte alignment when uploading texture data */
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        /* Clamping to edges is important to prevent artifacts when scaling */
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        /* Linear filtering usually looks best for text */
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        /* Paste all glyph bitmaps into the texture, remembering the offset */
        int ox = 0;
        int oy = 0;

        rowh = 0;

        for (int i = 0; i < TOTAL_CHAR; i++) {
        	if (FT_Load_Char(face, i, FT_LOAD_RENDER)) {
        		fprintf(stderr, "Loading character %c failed!\n", i);
        		continue;
        	}

        	if (ox + g->bitmap.width + 1 >= MAXWIDTH) {
        		oy += rowh + 1;
        		rowh = 0;
        		ox = 0;
        	}

        	glTexSubImage2D(GL_TEXTURE_2D, 0, ox, oy, g->bitmap.width, g->bitmap.rows, GL_ALPHA, GL_UNSIGNED_BYTE, g->bitmap.buffer);
        	_ch[i].ax = g->advance.x >> 6;
        	_ch[i].ay = g->advance.y >> 6;

        	_ch[i].bw = g->bitmap.width;
        	_ch[i].bh = g->bitmap.rows;

        	_ch[i].bl = g->bitmap_left;
        	_ch[i].bt = g->bitmap_top;

        	_ch[i].tx = ox / double(w);
        	_ch[i].ty = oy / double(h);

        	rowh = std::max(rowh, g->bitmap.rows);
        	ox += g->bitmap.width + 1;
        }
        fprintf(stderr, "Generated a %d x %d (%d kb) texture atlas\n", w, h, w * h / 1024);
    }

	~atlas() {
		glDeleteTextures(1, &TextureID);
	}
};
//--------------------------------------//
// end Atlas


std::shared_ptr<atlas> a48_ptr;
std::shared_ptr<atlas> a24_ptr;
std::shared_ptr<atlas> a12_ptr;




// Box vertex index
namespace rmLidarBoundingBox_ns{
    const GLuint box_idx_data[] = {
        0,1,2,
        1,2,3
    };
}



rmText3D_v2::rmText3D_v2(std::string _path_Assets_in){
    init_paths(_path_Assets_in);
    _num_vertex_idx_per_box = (3*2);
    _num_vertex_per_box = 4; // 8;
    _max_num_box = 500; // 100000;
    _max_num_vertex_idx = _max_num_box*(long long)(_num_vertex_idx_per_box);
    _max_num_vertex = _max_num_box*(long long)(_num_vertex_per_box);
    //
	Init();
}
void rmText3D_v2::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("Text3D_v2.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("Text3D_v2.fs.glsl"), GL_FRAGMENT_SHADER);
    // Link _program_ptr
	_program_ptr->LinkProgram();
    //

    // Cache uniform variable id
	uniforms.proj_matrix = glGetUniformLocation(_program_ptr->GetID(), "proj_matrix");
	uniforms.mv_matrix = glGetUniformLocation(_program_ptr->GetID(), "mv_matrix");
    uniforms.textColor = glGetUniformLocation(_program_ptr->GetID(), "textColor");
    uniforms.ref_point = glGetUniformLocation(_program_ptr->GetID(), "ref_point");

    // Init model matrices
	m_shape.model = glm::mat4(1.0);
    attach_pose_model_by_model_ref_ptr(m_shape.model); // For adjusting the model pose by public methods
    // The reference point of a text
    ref_point = glm::vec2(0.0f);

    //Load model to shader _program_ptr
	LoadModel();

}
void rmText3D_v2::LoadModel(){

    /*
    // FreeType
    //-----------------------------------------//
    FT_Library ft;
    // All functions return a value different than 0 whenever an error occurred
    if (FT_Init_FreeType(&ft))
        std::cout << "ERROR::FREETYPE: Could not init FreeType Library" << std::endl;
    // Load font as face
    FT_Face face;
    // if (FT_New_Face(ft, "fonts/arial.ttf", 0, &face))
    if (FT_New_Face(ft, "/usr/share/fonts/truetype/freefont/FreeSans.ttf", 0, &face))
    // if (FT_New_Face(ft, "FreeSans.ttf", 0, &face))
        std::cout << "ERROR::FREETYPE: Failed to load font" << std::endl;
    //-----------------------------------------//



    // Create texture atlasses for several font sizes
	a48_ptr = new atlas(face, 48);
	a24_ptr = new atlas(face, 24);
	a12_ptr = new atlas(face, 12);
    //



    // Destroy FreeType once we're finished
    //-----------------------------------------//
    FT_Done_Face(face);
    FT_Done_FreeType(ft);
    //-----------------------------------------//
    */

    /*
    // Create texture atlasses for several font sizes
	a48_ptr = new atlas(48);
	a24_ptr = new atlas(24);
	a12_ptr = new atlas(12);
    //
    */



    if (!a48_ptr){
        a48_ptr.reset(new atlas(48));
    }else{
        std::cout << "The atlas<" << a48_ptr->font_size << "> is already created.\n";
    }
    if (!a24_ptr){
        a24_ptr.reset(new atlas(24));
    }else{
        std::cout << "The atlas<" << a24_ptr->font_size << "> is already created.\n";
    }
    if (!a12_ptr){
        a12_ptr.reset(new atlas(12));
    }else{
        std::cout << "The atlas<" << a12_ptr->font_size << "> is already created.\n";
    }





    // GL things
    //----------------------------------------//
    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);

    glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glBufferData(GL_ARRAY_BUFFER, _max_num_vertex * sizeof(point), NULL, GL_DYNAMIC_DRAW);

    // glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), NULL);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(point), NULL);
    glEnableVertexAttribArray(0);


    // ebo
    glGenBuffers(1, &m_shape.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_shape.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, _max_num_vertex_idx * sizeof(GLuint), NULL, GL_STATIC_DRAW);
    // Directly assign data to memory of GPU
    //--------------------------------------------//
	GLuint * _idx_ptr = (GLuint *)glMapBufferRange(GL_ELEMENT_ARRAY_BUFFER, 0, _max_num_vertex_idx * sizeof(GLuint), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	size_t _idx_idx = 0;
    long long _offset_box = 0;
	for (size_t i = 0; i < _max_num_vertex_idx; i++)
	{
        _idx_ptr[i] = rmLidarBoundingBox_ns::box_idx_data[_idx_idx] + _offset_box;
        // std::cout << "_idx_ptr[" << i << "] = " << _idx_ptr[i] << "\n";
        _idx_idx++;
        if (_idx_idx >= _num_vertex_idx_per_box){
            _idx_idx = 0;
            _offset_box += _num_vertex_per_box;
        }
	}
	glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
    // m_shape.indexCount = _max_num_vertex_idx; //  1 * _num_vertex_idx_per_box; // ;
    //--------------------------------------------//



    // Close
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    //----------------------------------------//




    /*
    // test
    vector<text_billboard_data> data_test;
    for (size_t _k=0; _k < 1000; ++_k){
        data_test.push_back(
            text_billboard_data(
                "billboard #" + std::to_string(_k),
                glm::vec3( float(_k)*2.0f, 0.0f, 2.0f),
                glm::vec2(0.0f, 0.0f),
                1.5f,
                glm::vec3(0.6f)
            )
        );
        insert_text(data_test);
    }
    //
    */


}
void rmText3D_v2::Update(float dt){
    // Update the data (buffer variables) here
}
void rmText3D_v2::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}
void rmText3D_v2::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here


    // test
    static int _count = 0;
    insert_text( text2D_data( "Hello world: " + std::to_string(_count) + std::string("\nSecond line\n\tThird line\nABCDEFGabcdefg"), glm::vec2(0.0f), 1.0f, glm::vec3(1.0f,1.0f,0.0f) ) );
    /*
    for (size_t _k=0; _k < 1000; ++_k){
        insert_text( text2D_data("Text #" + std::to_string(_k) + ": " + std::to_string(_count), glm::vec2( 0.0f, float(_k))) );
    }
    */

    //
    insert_text( text3D_data("Text3D") );
    //
    // insert_text( text_billboard_data("The billboard" ));
    insert_text(
        text_billboard_data(
            "The billboard\nSecond line\nSeq: " + std::to_string(_count),
            glm::vec3( 0.0f, 0.0f, 0.0f),
            glm::vec2(0.0f, 0.0f),
            1.0f,
            glm::vec3(1.0f, 0.8f, 0.2f),
            ALIGN_X::RIGHT,
            ALIGN_Y::CENTER
        )
    );
    /*
    for (size_t _k=0; _k < 1000; ++_k){
        insert_text(
            text_billboard_data(
                "billboard #" + std::to_string(_k) + ": " + std::to_string(_count),
                glm::vec3( float(_k)*2.0f, 0.0f, 2.0f),
                glm::vec2(0.0f, 0.0f)
            )
        );
    }
    */


    //
    for (size_t _k=0; _k < 3; ++_k){
        insert_text(
            text_freeze_board_data(
                "I am freezed board #" + std::to_string(_k) + ": " + std::to_string(_count),
                glm::vec3( float(_k)*20.0f, 0.0f, 10.0f),
                glm::vec2(0.0f, 0.0f),
                24,
                glm::vec3(1.0f, 0.0f, 1.0f),
                ALIGN_X::CENTER,
                ALIGN_Y::CENTER
            )
        );
    }

    //
    _count++;
    //
}
void rmText3D_v2::Render(std::shared_ptr<ViewManager> _camera_ptr){
    glBindVertexArray(m_shape.vao);
	_program_ptr->UseProgram();
    // queues
    //--------------------------------//
    while (!text2D_queue.empty()){
        _draw_one_text2D(_camera_ptr, text2D_queue.front() );
        text2D_queue.pop();
    }
    while (!text3D_queue.empty()){
        _draw_one_text3D(_camera_ptr, text3D_queue.front() );
        text3D_queue.pop();
    }
    while (!text_billboard_queue.empty()){
        _draw_one_text_billboard(_camera_ptr, text_billboard_queue.front() );
        text_billboard_queue.pop();
    }
    while (!text_freeze_board_queue.empty()){
        _draw_one_text_freeze_board(_camera_ptr, text_freeze_board_queue.front() );
        text_freeze_board_queue.pop();
    }
    //--------------------------------//

    // buffers
    //--------------------------------//
    for (size_t i=0; i < text2D_buffer.size(); ++i){
        _draw_one_text2D(_camera_ptr, text2D_buffer[i] );
    }
    for (size_t i=0; i < text3D_buffer.size(); ++i){
        _draw_one_text3D(_camera_ptr, text3D_buffer[i] );
    }
    for (size_t i=0; i < text_billboard_buffer.size(); ++i){
        _draw_one_text_billboard(_camera_ptr, text_billboard_buffer[i] );
    }
    for (size_t i=0; i < text_freeze_board_buffer.size(); ++i){
        _draw_one_text_freeze_board(_camera_ptr, text_freeze_board_buffer[i] );
    }
    //--------------------------------//

    _program_ptr->CloseProgram();
}


// Different draw methods
//--------------------------------------------------------//
void rmText3D_v2::_draw_one_text2D(std::shared_ptr<ViewManager> &_camera_ptr, text2D_data &_data_in){
    // static int _count = 0;

    // m_shape.model = translateMatrix * rotateMatrix * scaleMatrix;
    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));

    // RenderText("Hello world: " + std::to_string(++_count) + std::string("\nSecond line\n\tThird line\nABCDEFGabcdefg"), a48_ptr, 0.0, 0.0, 1.0, 1.0, glm::vec3(1.0f, 1.0f, 0.0f));
    RenderText(
        _data_in.text,
        a48_ptr,
        _data_in.position_2D[0],
        _data_in.position_2D[1],
        _data_in.size,
        _data_in.size,
        _data_in.color,
        _data_in.align_x,
        _data_in.align_y
    );
    //--------------------------------//
}
void rmText3D_v2::_draw_one_text3D(std::shared_ptr<ViewManager> &_camera_ptr, text3D_data &_data_in){

    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, _data_in.pose_ref_point) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));
    //
    RenderText(
        _data_in.text,
        a48_ptr,
        -1*(_data_in.offset_ref_point_2D[0]),
        -1*(_data_in.offset_ref_point_2D[1]),
        _data_in.size,
        _data_in.size,
        _data_in.color,
        _data_in.align_x,
        _data_in.align_y
    );
    //--------------------------------//
}
void rmText3D_v2::_draw_one_text_billboard(std::shared_ptr<ViewManager> &_camera_ptr, text_billboard_data &_data_in){
    // Calculate model matrix
    glm::mat4 view_m = _camera_ptr->GetModelViewMatrix();
    view_m[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    glm::mat4 _model_m = glm::transpose(view_m);
    _model_m[3] = glm::vec4(_data_in.position_ref_point, 1.0f);
    //

    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, _model_m) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));
    //
    RenderText(
        _data_in.text,
        a48_ptr,
        -1*(_data_in.offset_ref_point_2D[0]),
        -1*(_data_in.offset_ref_point_2D[1]),
        _data_in.size,
        _data_in.size,
        _data_in.color,
        _data_in.align_x,
        _data_in.align_y
    );
    //--------------------------------//
}
void rmText3D_v2::_draw_one_text_freeze_board(std::shared_ptr<ViewManager> &_camera_ptr, text_freeze_board_data &_data_in){
    // Calculate model matrix
    glm::mat4 view_m = _camera_ptr->GetModelViewMatrix();
    //
    // glm::vec4 _ref_in_canonical = ( _camera_ptr->GetProjectionMatrix()*view_m*glm::vec4(_data_in.position_ref_point, 1.0f) );
    // GLfloat _z_ref = (_ref_in_canonical.z);
    GLfloat _z_ref = ( view_m*glm::vec4(_data_in.position_ref_point, 1.0f) ).z;
    // GLfloat _scale = _z_ref / (_camera_ptr->GetProjectionMatrix() )[2][3];
    GLfloat _scale = _z_ref / (_camera_ptr->GetProjectionMatrix() )[2][3] / ( (_camera_ptr->GetViewportSize())[1]/2 );
    //
    view_m[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    glm::mat4 _model_m = glm::transpose(view_m);
    _model_m[3] = glm::vec4(_data_in.position_ref_point, 1.0f);
    //

    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, _model_m) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));
    //
    RenderText(
        _data_in.text,
        a48_ptr,
        -1*(_data_in.offset_ref_point_2D[0]),
        -1*(_data_in.offset_ref_point_2D[1]),
        1.0*_scale*_data_in.size,
        1.0*_scale*_data_in.size,
        _data_in.color,
        _data_in.align_x,
        _data_in.align_y
    );
    //--------------------------------//
}
//--------------------------------------------------------//




//-----------------------------------------------//
// void rmText3D_v2::RenderText(const std::string &text, atlas *_atlas_ptr, float x, float y, float scale_x_in, float scale_y_in, glm::vec3 color) {
void rmText3D_v2::RenderText(
    const std::string &text,
    std::shared_ptr<atlas> &_atlas_ptr,
    float x_in,
    float y_in,
    float scale_x_in,
    float scale_y_in,
    glm::vec3 color,
    ALIGN_X align_x,
    ALIGN_Y align_y
){

    GLfloat scale_x = scale_x_in/GLfloat(_atlas_ptr->font_size);
    GLfloat scale_y = scale_y_in/GLfloat(_atlas_ptr->font_size);
    //
    glUniform3f( uniforms.textColor, color.x, color.y, color.z);

    // Calculate the bound
    float _max_w = 0;
    float _max_h = 0;


    //
    float x = x_in;
    float y = y_in;

	point coords[ 6 * text.size() ];
	int _idx_count = 0;
    int _valid_word_count = 0;

    //
    int _word_per_line = 0;
    int _max_word_per_line = 0;
    int _line_count = 1;

    // Iterate through all characters
    std::string::const_iterator p;
    for (p = text.begin(); p != text.end(); p++) {
        if (*p == '\n'){
            x = x_in;
            y -= _atlas_ptr->font_size * scale_y;
            //
            _line_count++;
            _word_per_line = 0;
            continue;
        }
        //
        _word_per_line++;
        if (_max_word_per_line < _word_per_line){
            _max_word_per_line = _word_per_line;
        }
        //
        if (*p == '\t'){
            x += (_atlas_ptr->_ch[' '].ax * scale_x)*8;
            continue;
        }
	// for (uint8_t * p = (const uint8_t *)text; *p; p++) {
		/* Calculate the vertex and texture coordinates */
		float x2 = x + _atlas_ptr->_ch[*p].bl * scale_x;
		float y2 = -y - _atlas_ptr->_ch[*p].bt * scale_y;
		float w = _atlas_ptr->_ch[*p].bw * scale_x;
		float h = _atlas_ptr->_ch[*p].bh * scale_y;

		/* Advance the cursor to the start of the next character */
		x += _atlas_ptr->_ch[*p].ax * scale_x;
		y += _atlas_ptr->_ch[*p].ay * scale_y;

		/* Skip glyphs that have no pixels */
		if (!w || !h) // For example: space - " "
			continue;

        /*
        Sequence of index
        1 - 2
        | / |
        3 - 4
        */
        // T1
		coords[_idx_count++] = (point) {
    		x2, -y2,
            _atlas_ptr->_ch[*p].tx, _atlas_ptr->_ch[*p].ty};
		coords[_idx_count++] = (point) {
    		x2+w, -y2,
            _atlas_ptr->_ch[*p].tx + _atlas_ptr->_ch[*p].bw / float(_atlas_ptr->w), _atlas_ptr->_ch[*p].ty};
		coords[_idx_count++] = (point) {
    		x2, -y2-h,
            _atlas_ptr->_ch[*p].tx, _atlas_ptr->_ch[*p].ty + _atlas_ptr->_ch[*p].bh / float(_atlas_ptr->h)};

        // T2
        /*
        coords[_idx_count++] = (point) {
    		x2+w, -y2,
            _atlas_ptr->_ch[*p].tx + _atlas_ptr->_ch[*p].bw / float(_atlas_ptr->w), _atlas_ptr->_ch[*p].ty};
		coords[_idx_count++] = (point) {
    		x2, -y2-h,
            _atlas_ptr->_ch[*p].tx, _atlas_ptr->_ch[*p].ty + _atlas_ptr->_ch[*p].bh / float(_atlas_ptr->h)};
        */
		coords[_idx_count++] = (point) {
    		x2+w, -y2-h,
            _atlas_ptr->_ch[*p].tx + _atlas_ptr->_ch[*p].bw / float(_atlas_ptr->w), _atlas_ptr->_ch[*p].ty + _atlas_ptr->_ch[*p].bh / float(_atlas_ptr->h)};

        if (_max_w < (x2+w)){
            _max_w = (x2+w);
        }
        if (_max_h < (y2+h)){
            _max_h = (y2+h);
        }
        //
        _valid_word_count++;

	}

    // ref_point

    // Method 1: too precise that it will vibrate when word change
    // ref_point = glm::vec2(_max_w/2.0f, -1*_max_h/2.0f);

    // Method 2: less precise, using word (per line) count and line count
    float _space_x = 2*(_atlas_ptr->_ch[' '].ax * scale_x);
    float _space_y = _atlas_ptr->font_size * scale_y;
    //
    switch (align_x){
        case ALIGN_X::LEFT:
            ref_point[0] = 0.0f;
            break;
        case ALIGN_X::CENTER:
            ref_point[0] = _max_word_per_line*_space_x/2.0f;
            break;
        case ALIGN_X::RIGHT:
            ref_point[0] = _max_w;
            break;
        default:
            ref_point[0] = 0.0f;
            break;
    }
    switch (align_y){
        case ALIGN_Y::TOP:
            ref_point[1] = 0.0f;
            break;
        case ALIGN_Y::CENTER:
            // ref_point[1] = -1*_line_count*_space_y/2.0f;
            ref_point[1] = -1*_max_h/2.0f;
            break;
        case ALIGN_Y::BUTTON:
            ref_point[1] = -1*_max_h;
            break;
        default:
            ref_point[1] = 0.0f;
            break;
    }
    // ref_point = glm::vec2(_max_word_per_line*_space_x/2.0f, -1*_line_count*_space_y/2.0f);
    glUniform2f( uniforms.ref_point, ref_point.x, ref_point.y);



    // Update content of VBO memory
    glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);


	// Draw all the character on the screen in one go
	// glBufferData(GL_ARRAY_BUFFER, sizeof coords, coords, GL_DYNAMIC_DRAW);


    // Directly assign data to memory of GPU
    //--------------------------------------------//
    point * vertex_ptr = (point *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _idx_count* sizeof(point), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    for (size_t i = 0; i < _idx_count; i++)
    {
        vertex_ptr[i] = coords[i];
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    //--------------------------------------------//

    // Activate corresponding render state
    glActiveTexture(GL_TEXTURE0);
	// Use the texture containing the atlas
	glBindTexture(GL_TEXTURE_2D, _atlas_ptr->TextureID);

    // Draw
	// glDrawArrays(GL_TRIANGLES, 0, _idx_count);
    glDrawElements(GL_TRIANGLES, _valid_word_count*_num_vertex_idx_per_box, GL_UNSIGNED_INT, 0); // Using ebo

    glBindTexture(GL_TEXTURE_2D, 0);

}
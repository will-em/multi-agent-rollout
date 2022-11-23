#include <iostream>
#include <regex>
#include <fstream>

#include <iomanip>
#include <sstream>



#define TILE "video/assets/empty_tile.svg"
#define WALL "video/assets/walled_tile.svg"
#define ROBOT "video/assets/robot.svg"
#define BOX "video/assets/box.svg"
#define HEADER_LINES 3

std::vector<std::tuple<float, float, bool>> get_robot_positions(int * matrix_1, int * matrix_2, float alpha, std::pair<int,int> * size, int n_robots);

int generate_image(std::string file_path, int frame_number, int * m_matrix, std::pair<int,int> * size);

int generate_interpolated_image(std::string file_path, int frame_number, int * m_matrix_1, int * m_matrix_2, float alpha, std::pair<int,int> * size, int n_robots);
int add_template(std::ofstream & output_file, std::pair<int,int> * scene_size);
int add_closure(std::ofstream & output_file);

int add_tile(std::ofstream & output_file, std::pair<float,float> position);
int add_robot(std::ofstream & output_file, std::pair<float,float> position, int index);
int add_box(std::ofstream & output_file, std::pair<float,float> position);
int add_wall(std::ofstream & output_file, std::pair<float,float> position);

int process_svg(
	std::string base_path,
	std::pair<float,float> position,
	std::string &out,
	std::string * substitutes,
	int n_substitutes);


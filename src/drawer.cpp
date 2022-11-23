#include "drawer.h"

int generate_image(std::string file_path, int frame_number, int * m_matrix, std::pair<int,int> * size) {
	// Generates a single frame of the current state, in the file
	// "{file_path}frame{frame_number}.svg".

	// Generate the complete filename with the path and the frame number given.
	// The frame number has a fixed length of 4 digits, using 0 for filling, so
	// frame 23 is named frame0023.
	std::stringstream ss;
	ss << std::setw(4) << std::setfill('0') << frame_number;
	std::string file_name = file_path + "frame" + ss.str() + ".svg";
	std::cout << file_name << std::endl;
	std::ofstream output_file(file_name);

	// Add the initial text of the svg file, including the size of the image.
	add_template(output_file, size);


	// Loop through each cell of the warehouse
	for (int x=0; x<size->first; x++){
		for (int y=0; y<size->second; y++){
			std::pair<int,int> tile_pos(x, y);

			int element = m_matrix[x + y * size->first];

			switch(element) {
				case 0:
					// An empty cell
					add_tile(output_file, tile_pos);
					break;
				case 1:
					// A wall
					add_wall(output_file, tile_pos);
					break;

				case 2:
					// A box on an empty file
					add_tile(output_file, tile_pos);
					add_box(output_file, tile_pos);
					break;

				case 3:
					// Should not happen
					break;
				
				default:
					// A robot on an empty tile. First draw the tile.
					add_tile(output_file, tile_pos);

					// Determine the number of the robot, and whether it is
					// carrying a box.
					bool carrying_box = element > 0;
					int robot_index;

					if (carrying_box) {
						robot_index = element - 3;
					} else {
						robot_index = -element - 3;
					}

					// Print the robot
					add_robot(output_file, tile_pos, robot_index);

					// If it is carrying a box, add it as well
					if (carrying_box) {
						add_box(output_file, tile_pos);
					}

			}
		}
	}

	// Add the final part of the svg file.
	add_closure(output_file);

	output_file.close();

	return 0;
}

int generate_interpolated_image(std::string file_path, int frame_number, int * m_matrix_1, int * m_matrix_2, float alpha, std::pair<int,int> * size, int n_robots) {
	// Similar to generate_image, but takes two frames and interpolates
	// positions between them. The parameter alpha indicates at which point of
	// the transition the interpolation takes place: 0.0 is the first frame,
	// 1.0 is the second frame.

	// Obtain the complete name of the file
	std::stringstream ss;
	ss << std::setw(4) << std::setfill('0') << frame_number;
	std::string file_name = file_path + "frame" + ss.str() + ".svg";
	std::ofstream output_file(file_name);


	add_template(output_file, size);


	for (int x=0; x<size->first; x++){
		for (int y=0; y<size->second; y++){
			std::pair<int,int> tile_pos(x, y);

			int element = m_matrix_1[x + y * size->first];

			switch(element) {
				case 0:
					add_tile(output_file, tile_pos);
					break;
				case 1:
					add_wall(output_file, tile_pos);
					break;

				case 2:
					add_tile(output_file, tile_pos);
					add_box(output_file, tile_pos);
					break;

				case 3:
					break;
				
				default:
					add_tile(output_file, tile_pos);
					break;
			}
		}
	}

	// Generates a vector for an entry for each robot, which includes its
	// position in x, in y and whether it is carrying a box initially.
	std::vector<std::tuple<float, float, bool>> robot_positions = get_robot_positions(m_matrix_1, m_matrix_2, alpha, size, n_robots);

	// Prints all the robots at the interpolated positions, with theirs
	// respective boxes
	for (int robot_index=0; robot_index < n_robots; robot_index++){
		std::pair<float, float> position(
				std::get<0>(robot_positions[robot_index]),
				std::get<1>(robot_positions[robot_index])
		);

		bool carrying_box = std::get<2>(robot_positions[robot_index]);
		add_robot(output_file, position, robot_index);

		if (carrying_box) {
			add_box(output_file, position);
		}
	}

	add_closure(output_file);

	output_file.close();

	return 0;
}

std::vector<std::tuple<float, float, bool>> get_robot_positions(
		int * matrix_1,
		int * matrix_2,
		float alpha,
		std::pair<int,int> * size,
		int n_robots) {

	std::vector<std::pair<float,float>> robot_positions_state_1(n_robots);
	std::vector<std::pair<float,float>> robot_positions_state_2(n_robots);

	std::vector<bool> carrying_box(n_robots);

	for (int x=0; x < size->first; x++){
		for (int y=0; y < size->second; y++) {
			int element_1 = matrix_1[x + size->first * y];
			int element_2 = matrix_2[x + size->first * y];

			std::pair<float,float> position((float) x, (float) y);

			if (element_1 >=4) {
				int robot_index = element_1 - 4;
				robot_positions_state_1[robot_index] = position;
				carrying_box[robot_index] = true;

			} else if (element_1 <= -4) {
				int robot_index = -element_1 - 4;
				robot_positions_state_1[robot_index] = position;
				carrying_box[robot_index] = false;
			}

			if (element_2 >=4) {
				int robot_index = element_2 - 4;
				robot_positions_state_2[robot_index] = position;

			} else if (element_2 <= -4) {
				int robot_index = -element_2 - 4;
				robot_positions_state_2[robot_index] = position;
			}
		}
	}

	std::vector<std::tuple<float,float,bool>> robot_positions(n_robots);

	for (int r_index=0; r_index < n_robots; r_index++) {
		std::pair<double,double> interpolated_position(
			robot_positions_state_1[r_index].first * (1.0-alpha) 
			+ robot_positions_state_2[r_index].first * alpha,
			robot_positions_state_1[r_index].second * (1.0-alpha) 
			+ robot_positions_state_2[r_index].second * alpha
		);

		std::get<0>(robot_positions[r_index]) = interpolated_position.first;
		std::get<1>(robot_positions[r_index]) = interpolated_position.second;
		std::get<2>(robot_positions[r_index]) = carrying_box[r_index];
	}

	return robot_positions;
}

int add_template(std::ofstream & output_file, std::pair<int,int> * scene_size) {
	std::stringstream dimension_block;

	dimension_block << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n\n";
	dimension_block << "<svg\n";
	dimension_block << "   width=\"" << 100*scene_size->first << "\"\n";
	dimension_block << "   height=\"" << 100*scene_size->second << "\"\n";
	dimension_block << "   viewBox=\"0 0 " << 100*scene_size->first << " " << 100*scene_size->second << "\"\n";
	dimension_block << "   version=\"1.1\"\n";
	dimension_block << "   id=\"svg5\">\n";

	output_file << dimension_block.str();

	return 0;
}

int add_closure(std::ofstream & output_file) {
	output_file << "</svg>" << std::endl;
	return 0;
}

int add_tile(std::ofstream & output_file, std::pair<float,float> position){
	/* Adds an empty tile in the output file, in the position given by
	 * the second argument
	*/

	std::string svg_text;

	int process_status = process_svg(TILE, position, svg_text, NULL, 0);

	if (process_status == 0) {
		output_file << svg_text;
		return 0;

	} else {
		return process_status;
	}
}

int add_robot(std::ofstream & output_file, std::pair<float,float> position, int index){
	std::string svg_text;

	std::string args[1] = { std::to_string(index) };

	int process_status = process_svg(ROBOT, position, svg_text, args, 1);

	if (process_status == 0) {
		output_file << svg_text;
		return 0;

	} else {
		return process_status;
	}
}

int add_box(std::ofstream & output_file, std::pair<float,float> position){
	std::string svg_text;

	int process_status = process_svg(BOX, position, svg_text, NULL, 0);

	if (process_status == 0) {
		output_file << svg_text;
		return 0;

	} else {
		return process_status;
	}
}

int add_wall(std::ofstream & output_file, std::pair<float,float> position){
	std::string svg_text;

	int process_status = process_svg(WALL, position, svg_text, NULL, 0);

	if (process_status == 0) {
		output_file << svg_text;
		return 0;

	} else {
		return process_status;
	}
}

int process_svg(
		std::string base_path,
		std::pair<float,float> position,
		std::string &out,
		std::string * substitutes,
		int n_substitutes){

	/* The function takes the svg given in the base path, modifies it and
	 * writes it into "out". The modification has two steps: first, it adds the
	 * coordinates x and y given in the position pair. Then, it replaces all
	 * ocurrences of {n}, where n is a number between 0 and 9, by the string
	 * given in substitutes[n].
	 */


	// Currently the limit of substitutes is 10, but it probably can be
	// increased.
	if (n_substitutes > 10) {
		perror("Too many substitutes");
		exit(-1);
	}

	// Open the svg template as a file stream, so it can be read line by line
	std::ifstream svg_file(base_path);
	if (svg_file.fail()){
		perror("Error with svg file");
		exit(-2);
	}

	// The substitution flag defines the sequence of characters that must be
	// replaced in the svg. The first definition is a placeholder, as X must be
	// substituted by the number of the substitution.
	std::string substitution_flag = "\\{X\\}";

	// The position block is a block of text to be pasted in the svg, to define
	// the position of the image. It takes the coordinates defined in the
	// argument.
	std::stringstream position_block;
	position_block << "   x=\"" << 100*position.first << "\"\n   y=\"" << 100*position.second << "\"\n";


	// The line number is tracked so the position_block can be pasted in the
	// correct place
	int line_number = 0;
	std::string line;

	while(std::getline(svg_file, line)) {
		line_number++;

		if (line_number <= HEADER_LINES) { continue; }

		// Once the line from the templace has been read, we first check for
		// all possible subtitution flags, and replace them by the
		// corresponding string
		for (int i = 0; i<n_substitutes; i++) {

			// Transforms {X} into {n}, where n is the number of the
			// substitute. It then replaced it using regex.
			substitution_flag[2] = i + '0';
			std::regex detector(substitution_flag);
			line = std::regex_replace(line, detector, substitutes[i]);
		}

		// Wirte the modified line into the final string.
		out.append(line);
		out.append("\n");

		// In the appropiate line, add the position_block as well
		if (line_number == 4) {
			out.append(position_block.str());
		}
	}

	svg_file.close();

	return 0;
}

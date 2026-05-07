#include "../include/LoadLasFile.h"

void loadLasFile(std::string inputPath, PointCloud3fPtr pointcloud)
{
	if (!std::filesystem::exists(inputPath)) {
		std::cout << "File path does not exist!!!" << std::endl;
		return;
	}
	//creat the reader
	laszip_POINTER reader;
	laszip_create(&reader);
	//open the reader
	laszip_BOOL isCompressed = 0;
	laszip_open_reader(reader, inputPath.c_str(), &isCompressed);

	laszip_header* header;
	laszip_get_header_pointer(reader, &header);

	//points number
	laszip_I64 numPoints = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

	//get a point to the points that will be read
	laszip_point* pointer;
	laszip_get_point_pointer(reader, &pointer);

	double scale_x = header->x_scale_factor;
	double scale_y = header->y_scale_factor;
	double scale_z = header->z_scale_factor;
	double offset_x = header->x_offset;
	double offset_y = header->y_offset;
	double offset_z = header->z_offset;

	laszip_I64 count = 0;

	Point3f point;

	while (count < numPoints)
	{
		laszip_read_point(reader);

		point.coords()[0] = pointer->X * scale_x + offset_x;
		point.coords()[1] = pointer->Y * scale_y + offset_y;
		point.coords()[2] = pointer->Z * scale_z + offset_z;

		pointcloud->addPoint(point);
		count++;
	}

	laszip_close_reader(reader);
	laszip_destroy(reader);
}

void outputLasFile(std::string outputPath, PointCloud3fPtr pointcloud)
{
	std::filesystem::path file_path(outputPath);
	std::filesystem::path parent = file_path.parent_path();

	if (!parent.empty() && !std::filesystem::exists(parent)) {
		std::filesystem::create_directories(parent);
	}

	laszip_I64 numPoints = pointcloud->size();

	Box3f box = pointcloud->boundingBox();

	//creat the writer
	laszip_POINTER writer;
	laszip_create(&writer);

	laszip_header* header;
	laszip_get_header_pointer(writer, &header);

	header->file_source_ID = 2048;
	header->global_encoding = 0;
	header->version_major = 1;
	header->version_minor = 2;
	header->header_size = 227;
	header->offset_to_point_data = 227;
	header->point_data_format = 1;
	header->point_data_record_length = 28;

	header->x_scale_factor = 0.001;
	header->y_scale_factor = 0.001;
	header->z_scale_factor = 0.001;
	header->x_offset = box.min().coords()[0];
	header->y_offset = box.min().coords()[1];
	header->z_offset = box.min().coords()[2];
	header->number_of_point_records = numPoints;


	//laszip_set_header(writer, header);

	//open the writer
	laszip_BOOL isCompressed = 0;
	laszip_open_writer(writer, outputPath.c_str(), isCompressed);

	laszip_point* point = new laszip_point;
	laszip_get_point_pointer(writer, &point);


	laszip_I64 count = 0;
	while (count < numPoints)
	{
		point->X = static_cast<laszip_I64>(std::round((pointcloud->points()[count].coords()[0] - header->x_offset) / header->x_scale_factor));
		point->Y = static_cast<laszip_I64>(std::round((pointcloud->points()[count].coords()[1] - header->y_offset) / header->y_scale_factor));
		point->Z = static_cast<laszip_I64>(std::round((pointcloud->points()[count].coords()[2] - header->z_offset) / header->z_scale_factor));

		//laszip_set_coordinates(writer, XYZ);
		//laszip_set_point(writer, point);
		laszip_write_point(writer);
		count++;
	}


	laszip_close_writer(writer);
	laszip_destroy(writer);
}

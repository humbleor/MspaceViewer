#include <iostream>
#include "../include/RegistrationU2T.h"
#include "../include/LoadLasFile.h"

int main()
{
	float resolution = 0.2f;// GridMinimum体素格的边长
	float setp = 0.5f;// ULS keypnt提取格网边长
	float radius = 5.0f;//邻域半径
	float rSetp = 0.5f;//径向距离 间隔
	int numsSectors = 360;//0-360的极坐标范围 分为360份
	float angleThe = 2.0;//角度差
	float a2DThe = 0.1;//维度特征2d 差
	float a3DThe = 0.1;//维度特征3d 差

	PointCloud3fPtr source = std::make_shared<PointCloud3f>();
	PointCloud3fPtr target = std::make_shared<PointCloud3f>();
	loadLasFile("UAV.las", source);
	loadLasFile("TLS2.las", target);

	std::shared_ptr<RegistrationU2T> u2t = std::make_shared<RegistrationU2T>(source, target);
	u2t->setGridFilterRes(resolution);
	u2t->gridStep(setp);
	u2t->setSearchRadius(radius);
	u2t->setRadiusStep(rSetp);
	u2t->setNumSectors(numsSectors);
	u2t->descriptorsThreshold(angleThe, a2DThe, a3DThe);
	u2t->registration();
	std::array<std::array<float, 4>, 4> transMatrix = u2t->getTranslationMatrix();
	std::cout << "transformation matrix:" << std::endl;
	std::cout << transMatrix[0][0] << " " << transMatrix[0][1] << " " << transMatrix[0][2] << " " << transMatrix[0][3] << std::endl;
	std::cout << transMatrix[1][0] << " " << transMatrix[1][1] << " " << transMatrix[1][2] << " " << transMatrix[1][3] << std::endl;
	std::cout << transMatrix[2][0] << " " << transMatrix[2][1] << " " << transMatrix[2][2] << " " << transMatrix[2][3] << std::endl;
	std::cout << transMatrix[3][0] << " " << transMatrix[3][1] << " " << transMatrix[3][2] << " " << transMatrix[3][3] << std::endl;
	return 0;
}
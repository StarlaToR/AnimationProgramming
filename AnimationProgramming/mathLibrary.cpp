
/* Library of math function. Library made by Bryan Bachelet && Omaya Lise */

#include <cstdio>
#include <cstdlib>
#define _USE_MATH_DEFINES 
#include <cmath>
#include <cassert>
#include <ctime>
#include "mathLibrary.hpp"




const Vector2D Vector2D::zero = { 0, 0 };
const Vector2D Vector2D::up = { 0, 1 };
const Vector2D Vector2D::left = { -1, 0 };
const Vector2D Vector2D::down = { 0, -1 };
const Vector2D Vector2D::right = { 1, 0 };

Vector2D::Vector2D()
{
	x = 0;
	y = 0;
}

Vector2D::~Vector2D()
{
}
Vector2D::Vector2D(float x, float y)
{
	this->x = x;
	this->y = y;
}
Vector2D::Vector2D(int x, int y)
{
	this->x = (float)x;
	this->y = (float)y;
}
Vector2D::Vector2D(int number)
{
	x = (float)number;
	y = (float)number;
}

// Get angle of degree from a vector to a another.
// The angle is between 0 and 180 degree
float GetDegreAngle(Vector2D a, Vector2D b)
{
	a.normalize();
	b.normalize();
	float dot = a.DotProduct2D(b);
	return radToDeg(acos(dot));
}

// Get angle of degree from a vector to a another.
// The angle is between -180 and 180 degree
float GetSignedDegreAngle(Vector2D a, Vector2D b)
{
	float angle = atan2(b.y, b.x) - atan2(a.y, a.x);
	return radToDeg(angle);
}

// Rotate a vector;
void RotateVector2D(Vector2D* direction, float angle)
{
	angle = (float)degToRad(angle);
	Vector2D startVector = *direction;
	direction->x = startVector.x * cosf(angle) - (startVector.y * sinf(angle));
	direction->y = startVector.x * sinf(angle) + (startVector.y * cosf(angle));
}

// ---------- Algebric Function -----------------

// Make transition between the start value and the end value
float LerpFloat(float start, float end, float t)
{
	return ((end - start) * t) + start;
}

Vector2D LerpVector(Vector2D start, Vector2D end, float t)
{
	return ((end - start) * t) + start;
}

//Limit the value of a variable between a max and a min
float ClampFloat(float number, float min, float max)
{
	number = number > max ? max : number;
	number = number < min ? min : number;
	return number;
}
int ClampInt(int number, int min, int max)
{
	number = number > max ? max : number;
	number = number < min ? min : number;
	return number;
}

Quaternion Slerp(Quaternion start, Quaternion end, float t)
{
	ClampFloat(t, 0.f, 1.f);

	float dotproduct = start.w * end.w + start.x * end.x + start.y * end.y + start.z * end.z;

	if (dotproduct < 0)
	{
		end *= -1;
		dotproduct *= -1;
	}

	float absdotproduct = abs(dotproduct);
	float theta = acos(dotproduct);

	float multiplier1, multiplier2;
	if (1 - absdotproduct < 0.0001f)
	{
		// Linear interpolation for close orientations.
		multiplier1 = 1 - t;
		multiplier2 = t;
	}
	else
	{
		multiplier1 = sin((1 - t) * theta) / sin(theta);
		multiplier2 = sin(t * theta) / sin(theta);
	}


	Quaternion quat = start * multiplier1 + end * multiplier2;
	return quat.GetNormalizedQuat();
}

int32_t GetSeed()
{
	time_t currentTime;
	struct tm* localTime = new tm();
	time(&currentTime);
	localtime_s(localTime, &currentTime);
	int32_t seed = localTime->tm_min * localTime->tm_sec * localTime->tm_hour;
	delete(localTime);
	return seed;
}

int32_t getRandomNumber(int32_t state)
{
	uint64_t x = (uint64_t)state;
	int m = (int)pow(2, 31);
	int a = 1103515245;
	int c = 12345;
	uint64_t result = (a * x + c) % m; // La formule

	return (int32_t)result;
}

int RandomNumber(int32_t& state, int min, int max)
{
	state = getRandomNumber(state);
	int number = state % (max - 1 - min);
	number += min;
	return number;
}

float RandomNumber(int32_t& state, float min, float max)
{
	float random = ((float)rand()) / (float)RAND_MAX;
	float diff = max - min;
	float r = random * diff;
	return min + r;
}

//--------------- Vector 3 ------------ //

Vector3D::Vector3D()
{
	x = 0.0f;
	y = 0.0f;
	z = 0.0f;
}
Vector3D::Vector3D(const float c1, const float c2, const float c3)
{
	x = c1;
	y = c2;
	z = c3;
}





Vector3D::~Vector3D()
{
}

const Vector3D Vector3D::zero = { 0.0f, 0.0f, 0.f };
const Vector3D Vector3D::up = { 0.0f, 1.0f, 0.f };
const Vector3D Vector3D::down = { 0.f, -1.f, 0.f };
const Vector3D Vector3D::left = { -1.0f, 0.f, 0.f };
const Vector3D Vector3D::right = { 1.f, 0.f, 0.f };
const Vector3D Vector3D::forward = { 0.f, 0.f, 1.f };
const Vector3D Vector3D::back = { 0.f, 0.f, -1.f };

void Vector3D::ToString(const float* coord, const char* name)
{
	std::cout << name << "\n"
		<< std::endl;

	std::cout << "[";
	for (int i = 0; i < 3; i++)
	{
		std::cout << " " << coord[i];
	}
	std::cout << " ]" << std::endl;


	std::cout << "\n"
		<< std::endl;
}

// --------------- Vector 4 ------------ //

const Vector4D Vector4D::zero = { 0.0f, 0.0f, 0.f, 1.0f };
const Vector4D Vector4D::up = { 0.0f, 1.0f, 0.f, 1.0f };
const Vector4D Vector4D::down = { 0.f, -1.f, 0.f, 1.0f };
const Vector4D Vector4D::left = { -1.0f, 0.f, 0.f, 1.0f };
const Vector4D Vector4D::right = { 1.f, 0.f, 0.f, 1.0f };
const Vector4D Vector4D::forward = { 0.f, 0.f, 1.f, 1.0f };
const Vector4D Vector4D::back = { 0.f, 0.f, -1.f, 1.0f };

Vector4D::Vector4D(/* args */)
{
	x = 0;
	y = 0;
	z = 0;
	w = 0;
}

Vector4D::Vector4D(float _x, float _y, float _z, float _w)
{
	x = _x;
	y = _y;
	z = _z;
	w = _w;
}
Vector4D::Vector4D(const Vector3D& vec3, const float _w)
{
	x = vec3.x;
	y = vec3.y;
	z = vec3.z;
	w = _w;
}

Vector4D::~Vector4D()
{
}

// -----------------  Transform ----------------------

namespace Physics
{

	Transform::Transform(/* args */)
	{
	}

	Transform::Transform(Vector3D& pos)
	{
		position = pos;
	}
	Transform::Transform(Vector3D& pos, Vector3D& rot)
	{
		position = pos;
		rotation = rot;
	}
	Transform::Transform(Vector3D pos, Quaternion r)
	{
		position = pos;
		rot = r;
	}
	Transform::Transform(Vector3D& pos, Vector3D& rot, Vector3D& sca)
	{
		position = pos;
		rotation = rot;
		scale = sca;
	}
	Transform::~Transform()
	{
	}
}

// ------------------------- Matrix 4D -----------------------

Matrix4D::Matrix4D()
{
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			coord[i][j] = 0.0f;
		}
	}
}

Matrix4D::Matrix4D(float tab[4][4])
{
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			coord[i][j] = tab[i][j];
		}
	}
}

Matrix4D::Matrix4D(float tab[16])
{
	for (int j = 0; j < 16; j++)
	{
		e[j] = tab[j];
	}
}


Matrix4D::~Matrix4D()
{
}

Matrix4D Matrix4D::operator+(const Matrix4D m2)
{
	Matrix4D result;
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			result.coord[j][i] = coord[j][i] + m2.coord[j][i];
		}
	}

	return result;
}

Matrix4D Matrix4D::operator-(const Matrix4D m2)
{
	Matrix4D result;
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			result.coord[j][i] = coord[j][i] - m2.coord[j][i];
		}
	}

	return result;
}

Vector3D Matrix4D::operator*(Vector3D _v2)
{
	Vector4D v2 = { _v2.x, _v2.y, _v2.z, 1.0f };
	Vector4D result = { 0.0f, 0.0f, 0.f, 0.0f };
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			result.coord[j] += coord[i][j] * v2.coord[i];
		}
	}

	Vector3D res = { result.x, result.y, result.z };
	return res;
}

Vector4D Matrix4D::operator*(Vector4D v2)
{
	Vector4D result = { 0.0f, 0.0f, 0.f, 0.0f };
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			result.coord[j] += coord[i][j] * v2.coord[i];
		}
	}

	return result;
}

Matrix4D Matrix4D::operator*(const Matrix4D m2)
{
	Matrix4D multMatrix;

	for (int k = 0; k < 4; k++)
	{
		for (int j = 0; j < 4; j++)
		{
			for (int i = 0; i < 4; i++)
			{
				multMatrix.coord[j][k] += coord[i][k] * m2.coord[j][i];
			}
		}
	}

	return multMatrix;
}

void Matrix4D::LoadIdentity()
{
	for (int i = 0; i < 4; i++)
	{
		coord[i][i] = 1.0f;
	}
}

Matrix4D Matrix4D::CreateTranslationMatrix(const Vector3D& translation)
{
	Matrix4D transMatrix;
	transMatrix.LoadIdentity();

	transMatrix.coord[3][0] = translation.x;
	transMatrix.coord[3][1] = translation.y;
	transMatrix.coord[3][2] = translation.z;

	return transMatrix;
}

Matrix4D Matrix4D::CreateScaleMatrix(const Vector3D& scale)
{
	Matrix4D scaled;
	scaled.LoadIdentity();
	scaled.coord[0][0] = scale.x;
	scaled.coord[1][1] = scale.y;
	scaled.coord[2][2] = scale.z;

	return scaled;
}

Matrix4D Matrix4D::CreateXRotationMatrix(float angle)
{
	Matrix4D rotXMat;
	rotXMat.LoadIdentity();
	angle = angle * (float)M_PI / 180.0f;
	rotXMat.coord[1][1] = cosf(angle);
	rotXMat.coord[1][2] = sinf(angle);
	rotXMat.coord[2][1] = -sinf(angle);
	rotXMat.coord[2][2] = cosf(angle);

	return rotXMat;
}

Matrix4D Matrix4D::CreateYRotationMatrix(float angle)
{
	Matrix4D rotYMat;
	rotYMat.LoadIdentity();
	angle = angle * (float)M_PI / 180.0f;
	rotYMat.coord[0][0] = cosf(angle);
	rotYMat.coord[0][2] = -sinf(angle);
	rotYMat.coord[2][0] = sinf(angle);
	rotYMat.coord[2][2] = cosf(angle);

	return rotYMat;
}

Matrix4D Matrix4D::CreateZRotationMatrix(float angle)
{
	Matrix4D rotZMat;
	rotZMat.LoadIdentity();
	angle = angle * (float)M_PI / 180.0f;
	rotZMat.coord[0][0] = cosf(angle);
	rotZMat.coord[1][0] = -sinf(angle);
	rotZMat.coord[0][1] = sinf(angle);
	rotZMat.coord[1][1] = cosf(angle);

	return rotZMat;
}


Matrix4D Matrix4D::CreateRotationMatrix(Vector3D angle)
{
	Matrix4D transfMatrix;
	transfMatrix.LoadIdentity();
	transfMatrix = transfMatrix * CreateYRotationMatrix(angle.y);
	transfMatrix = transfMatrix * CreateXRotationMatrix(angle.x);
	transfMatrix = transfMatrix * CreateZRotationMatrix(angle.z);
	return transfMatrix;
}

Matrix4D Matrix4D::CreateTransformMatrix(Physics::Transform& trans)
{
	Matrix4D transfMatrix = Matrix4D();
	transfMatrix = CreateTranslationMatrix(trans.position);
	transfMatrix = transfMatrix * GetMatrixFromQuaternion(trans.rot);
	transfMatrix = transfMatrix * CreateScaleMatrix(trans.scale);
	return transfMatrix;
}

Matrix4D Matrix4D::CreateTransformMatrix(const Vector3D position, const Quaternion rotation, const Vector3D scale)
{
	Matrix4D transfMatrix = Matrix4D();
	transfMatrix = CreateTranslationMatrix(position);
	transfMatrix = transfMatrix * GetMatrixFromQuaternion(rotation);
	transfMatrix = transfMatrix * CreateScaleMatrix(scale);
	return transfMatrix;
}


Vector3D Matrix4D::GetPositionFromMatrix(Matrix4D mat)
{
	Vector3D position;

	position.x = mat.coord[3][0];
	position.y = mat.coord[3][1];
	position.z = mat.coord[3][2];

	return position;
}


Vector3D Matrix4D::GetScaleFromMatrix(Matrix4D mat)
{
	Vector3D scale;
	Matrix3D mat3 = Matrix3D(mat);
	ExtractScaleFromMatrix(mat3, scale);

	return scale;
}

bool closeEnough(const float& a, const float& b, const float& epsilon = std::numeric_limits<float>::epsilon()) {
	return (epsilon > std::abs(a - b));
}

void ExtractScaleFromMatrix(Matrix3D& mat)
{
	for (int i = 0; i < 3; i++)
	{
		Vector3D xScale = Vector3D(mat.coord[0][i], mat.coord[1][i], mat.coord[2][i]);
		float length = xScale.GetMagnitude();
		mat.coord[0][i] = mat.coord[0][i] / length;
		mat.coord[1][i] = mat.coord[1][i] / length;
		mat.coord[2][i] = mat.coord[2][i] / length;
	}

}

void ExtractScaleFromMatrix(Matrix3D& mat, Vector3D& scale)
{
	for (int i = 0; i < 3; i++)
	{
		Vector3D xScale = Vector3D(mat.coord[0][i], mat.coord[1][i], mat.coord[2][i]);
		float length = xScale.GetMagnitude();
		scale.coord[i] = xScale.GetMagnitude();
		mat.coord[0][i] = mat.coord[0][i] / length;
		mat.coord[1][i] = mat.coord[1][i] / length;
		mat.coord[2][i] = mat.coord[2][i] / length;
	}

}

Vector3D Matrix4D::GetRotationFromMatrix(Matrix4D mat)
{
	Vector3D angle;
	Matrix3D mat3 = Matrix3D(mat);

	ExtractScaleFromMatrix(mat3); // extract Scale 

	float  thetaX = 0, thetaY = 0, thetaZ = 0;
	if (mat3.coord[1][2] < 1)
	{
		if (mat3.coord[1][2] > -1)
		{
			thetaX = asin(-mat3.coord[1][2]);
			thetaY = atan2(mat3.coord[0][2], mat3.coord[2][2]);
			thetaZ = atan2(mat3.coord[1][0], mat3.coord[1][1]);
		}
		else
		{
			thetaX = M_PI / 2.0f;
			thetaY = -atan2(-mat3.coord[0][1], mat3.coord[0][0]);
			thetaZ = 0;
		}
	}
	else
	{
		thetaX = -M_PI / 2.0f;
		thetaY = atan2(-mat3.coord[0][1], mat3.coord[0][0]);
		thetaZ = 0;
	}

	angle.x = radToDeg(thetaX);
	angle.y = radToDeg(thetaY);
	angle.z = radToDeg(thetaZ);
	return angle;

}

Physics::Transform Matrix4D::GetTransformFromMatrix(Matrix4D mat, Physics::Transform& transform)
{
	transform.rot = GetQuaternionFromMatrix(mat);
	//transform.scale = GetScaleFromMatrix(mat);
	transform.position = GetPositionFromMatrix(mat);

	return transform;
}

Matrix4D Matrix4D::TransposeMatrix(Matrix4D matrix)
{
	Matrix4D matTransp;

	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			matTransp.coord[i][j] = matrix.coord[j][i];
		}
	}
	return matTransp;
}

float Matrix4D::Determinant2x2(Matrix4D matrix)
{
	float det = 0.0f;
	det = matrix.coord[0][0] * matrix.coord[1][1] - matrix.coord[0][1] * matrix.coord[1][0];
	return det;
}

Matrix4D Matrix4D::Mat3x3ToMat2x2(int row, int columns, Matrix4D mat3x3)
{
	Matrix4D mat2x2;
	int index = 0;
	for (int j = 0; j < 3; j++)
	{
		if (j != columns)
		{
			for (int i = 0; i < 3; i++)
			{
				if (i != row)
				{
					mat2x2.coord[index / 2][index % 2] = mat3x3.coord[j][i];
					index++;
				}
			}
		}
	}

	return mat2x2;
}

int Matrix4D::MatrixSignElement(int index)
{
	if (index % 2 == 0)
		return 1;
	else
		return -1;
}

float Matrix4D::Determinant3x3(Matrix4D matrix)
{
	float det = 0.0f;

	for (int j = 0; j < 3; j++)
	{
		Matrix4D mat2x2 = Matrix4D::Mat3x3ToMat2x2(0, j, matrix);
		det += MatrixSignElement(j) * matrix.coord[j][0] * Determinant2x2(mat2x2);
	}
	return det;
}

Matrix4D Matrix4D::Mat4x4ToMat3x3(int row, int columns, Matrix4D mat4x4)
{

	Matrix4D mat3x3;
	int index = 0;
	for (int j = 0; j < 4; j++)
	{
		if (j != columns)
		{
			for (int i = 0; i < 4; i++)
			{
				if (i != row)
				{
					mat3x3.coord[index / 3][index % 3] = mat4x4.coord[j][i];
					index++;
				}
			}
		}
	}

	return mat3x3;
}

float Matrix4D::Determinant4x4(Matrix4D matrix)
{
	float det = 0.0f;

	for (int j = 0; j < 4; j++)
	{
		Matrix4D mat3x3 = Matrix4D::Mat4x4ToMat3x3(0, j, matrix);
		det += MatrixSignElement(j) * matrix.coord[j][0] * Determinant3x3(mat3x3);
	}
	return det;
}

Matrix4D Matrix4D::MatrixCofacteur(Matrix4D mat4x4)
{
	Matrix4D mat4x4Cof;
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			Matrix4D mat3x3 = Matrix4D::Mat4x4ToMat3x3(i, j, mat4x4);
			mat4x4Cof.coord[j][i] = Matrix4D::MatrixSignElement(j + i) * Determinant3x3(mat3x3);
		}
	}

	return mat4x4Cof;
}

Matrix4D Matrix4D::InverseMatrix(Matrix4D matrix)
{
	Matrix4D matInverse;
	Matrix4D mat4x4 = Matrix4D::MatrixCofacteur(matrix);
	mat4x4 = TransposeMatrix(mat4x4);
	float det = Matrix4D::Determinant4x4(matrix);
	for (int j = 0; j < 4; j++)
	{
		for (int i = 0; i < 4; i++)
		{
			matInverse.coord[j][i] = (1 / det) * mat4x4.coord[j][i];
		}
	}
	return matInverse;
}

void Matrix4D::ToString(const float* coord, const char* name)
{
	std::cout << name << "\n"
		<< std::endl;
	for (int j = 0; j < 4; j++)
	{
		std::cout << "[";
		for (int i = 0; i < 4; i++)
		{
			std::cout << " " << coord[i + j * 4];
		}
		std::cout << " ]" << std::endl;
	}

	std::cout << "\n"
		<< std::endl;
}

Matrix4D Matrix4D::GetMatrixFromQuaternion(Quaternion quat)
{

	float a = quat.coord[0];
	float b = quat.coord[1];
	float c = quat.coord[2];
	float d = quat.coord[3];
	float tab[4][4] = {
		{2.f * (a * a + b * b) - 1.0f,	2.f * (b * c + d * a),			2.f * (b * d - c * a),		  0},
		{2.f * (b * c - d * a),			2.f * (a * a + c * c) - 1.0f,	2.f * (c * d + b * a),		  0},
		{2.f * (b * d + c * a),			2.f * (c * d - b * a),			2.f * (a * a + d * d) - 1.0f, 0},
		{0,								0,								0,							  1.0f}
	};
	Matrix4D mat = Matrix4D(tab);

	return mat;
}

Quaternion Matrix4D::GetQuaternionFromMatrix(Matrix4D mat)
{
	float a, b, c, d;
	a = sqrtf(1 + mat.coord[0][0] + mat.coord[1][1] + mat.coord[2][2]) / 2.0f;
	b = (mat.coord[1][2] - mat.coord[2][1]) / (4.0f * a);
	c = (mat.coord[2][0] - mat.coord[0][2]) / (4.0f * a);
	d = (mat.coord[0][1] - mat.coord[1][0]) / (4.0f * a);

	return Quaternion(a, b, c, d);
}


Matrix3D::Matrix3D()
{
	for (int j = 0; j < 3; j++)
	{
		for (int i = 0; i < 3; i++)
		{
			coord[i][j] = 0.0f;
		}
	}
}

Matrix3D::~Matrix3D()
{
}


Matrix3D::Matrix3D(Matrix4D mat)
{
	for (int j = 0; j < 3; j++)
	{
		for (int i = 0; i < 3; i++)
		{
			coord[i][j] = mat.coord[i][j];
		}
	}
}


void Matrix3D::ToString(const float* coord, const char* name)
{
	for (int j = 0; j < 3; j++)
	{
		std::cout << "[";
		for (int i = 0; i < 3; i++)
		{
			std::cout << " " << coord[i + j * 3];
		}
		std::cout << " ]" << std::endl;
	}

	std::cout << "\n"
		<< std::endl;
}

Quaternion::Quaternion()
{
	this->scalar = 0;
	this->imaginaries = Vector3D();
}

Quaternion::Quaternion(float s, Vector3D i)
{
	this->scalar = s;
	this->imaginaries = i;
}

Quaternion::Quaternion(float w, float x, float y, float z)
{
	this->scalar = w;
	this->imaginaries = Vector3D(x, y, z);
}

Quaternion::~Quaternion()
{

}

Quaternion Quaternion::GetAngleAxis()
{
	Quaternion q = Quaternion();
	q.scalar = 2 * acos(this->scalar);
	q.imaginaries = Vector3D(this->imaginaries.x / sqrtf(1 - pow(this->scalar, 2.f)), this->imaginaries.y / sqrtf(1 - pow(this->scalar, 2.f)), this->imaginaries.z / sqrtf(1 - pow(this->scalar, 2.f)));
	return q;
}

Vector3D Quaternion::AngleAxis(Vector3D vector, float angle, Vector3D axis)
{
	Vector3D v = vector * cos(angle) + CrossProduct(axis, vector) * sin(angle) + axis * DotProduct(axis, vector) * (1 - cos(angle));
	return v;
}

void Quaternion::ToString(const float* coord, const char* name)
{
	std::cout << name << " [ ";
	for (int i = 0; i < 4; i++)
	{
		std::cout << coord[i] << " ";
	}
	std::cout << "]" << std::endl;

	std::cout << "\n" << std::endl;
}

Vector3D CrossProduct(Vector3D a, Vector3D b)
{
	Vector3D v;
	v.coord[0] = a.coord[1] * b.coord[2] - b.coord[1] * a.coord[2];
	v.coord[1] = a.coord[2] * b.coord[0] - b.coord[2] * a.coord[0];
	v.coord[2] = a.coord[0] * b.coord[1] - b.coord[0] * a.coord[1];
	return v;
}
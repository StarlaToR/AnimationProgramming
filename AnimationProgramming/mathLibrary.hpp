#pragma once
#include <stdint.h>
#include <math.h>
#define _USE_MATH_DEFINES 
#include <cmath>

#define degToRad(angleInDegrees) ((angleInDegrees) * (float)(M_PI / 180.0))
#define radToDeg(angleInRadians) ((angleInRadians) * (float)(180.0 / M_PI))
// --------------- Struct --------------



class Vector2D
{
private:
public:
	union
	{
		struct
		{
			float x, y;
		};

		float coord[2];
	};
	Vector2D();
	Vector2D(int number);
	Vector2D(float x, float y);
	Vector2D(int x, int y);
	~Vector2D();

	Vector2D operator+(Vector2D const b);
	Vector2D operator-(Vector2D const b);
	Vector2D operator*(float const multiplier);
	const Vector2D operator*(float const multiplier) const;
	Vector2D operator/(float const divider);
	Vector2D operator-();
	Vector2D operator+=(Vector2D const v2);
	Vector2D operator-=(Vector2D const v2);
	bool operator==(Vector2D const v2);
	bool operator==(Vector2D const v2) const;
	bool operator!=(Vector2D const v2);

	// operator Vector2() { return {x,y}; }

	void normalize(); // Get the direction of vector with two point
	float magnitude() const;
	Vector2D normal();

	Vector2D GetDirection2D(Vector2D PosB);
	float DotProduct2D(Vector2D v2);
	float GetDeterminant2D(Vector2D b);

	const static Vector2D zero;
	const static Vector2D up;
	const static Vector2D left;
	const static Vector2D down;
	const static Vector2D right;
};





class Vector3D
{
private:
	/* data */
public:
	// Propertiesz
	union
	{
		struct
		{
			float x, y, z;
		};

		float coord[3];
	};

	//Constuctors
	Vector3D();
	Vector3D(const float c1, const float c2, const float c3);
	~Vector3D();


	float GetMagnitude() const;
	void Normalize();


	bool operator==(const Vector3D v2);
	Vector3D operator+(const Vector3D v2);
	Vector3D operator-(const Vector3D v2);
	Vector3D operator*(const float multiplier);
	Vector3D operator+=(const Vector3D v2);
	Vector3D operator*(const float multiplier) const;

	void ToString(const float* coord, const char* name);

	const static Vector3D zero;
	const static Vector3D up;
	const static Vector3D left;
	const static Vector3D down;
	const static Vector3D right;
	const static Vector3D forward;
	const static Vector3D back;
};

union  Coord4
{
	struct
	{
		float x, y, z, w;
	};

	float coord[4];
};

class Vector4D
{
private:
	/* data */
public:
	union
	{
		struct
		{
			float x, y, z, w;
		};

		float coord[4];
	};


	Vector4D(/* args */);
	Vector4D(const Vector3D& vec3, float _w = 1.0f);
	Vector4D(float _x, float _y, float _z, float _w = 1.0f);
	~Vector4D();

	void Homogenize();
	float GetMagnitude() const;
	void Normalize();

	Vector4D& operator=(const Vector3D v2);
	Vector4D operator+(const Vector4D v2);
	Vector4D operator-(const Vector4D v2);
	Vector4D operator*(float const multiplier);

	operator Vector3D() const
	{
		return Vector3D(x, y, z);
	}

	const static Vector4D zero;
	const static Vector4D up;
	const static Vector4D left;
	const static Vector4D down;
	const static Vector4D right;
	const static Vector4D forward;
	const static Vector4D back;
};

using Color = Vector4D;

//Quaternions
class Quaternion
{
public:
	union
	{
		struct
		{
			float scalar;
			Vector3D imaginaries;
		};
		struct
		{
			float w, x, y, z;
		};
		float coord[4];
	};


	Quaternion();
	Quaternion(float s, Vector3D i);
	Quaternion(float w, float x, float y, float z);
	~Quaternion();

	Quaternion operator+(const Quaternion m2);
	void operator+=(const Quaternion m2);
	Quaternion operator-(const Quaternion m2);
	void operator-=(const Quaternion m2);
	Quaternion operator*(const Quaternion m2);
	void operator*=(const Quaternion m2);
	Quaternion operator*(float k);
	void operator*=(float k);
	Vector3D operator*(Vector3D v2);

	void ToString(const float* coord, const char* name);

	float GetNorm();
	void Normalize();
	Quaternion GetNormalizedQuat();
	void Conjugate();
	Quaternion GetConjugatedQuat();
	void Inverse();
	Quaternion GetInverseQuat();

	Quaternion GetAngleAxis();
	Vector3D AngleAxis(Vector3D vector, float angle, Vector3D axis);
private:

};

namespace Physics
{

	class Transform
	{
	private:
		/* data */
	public:
		Vector3D position = { 0.f, 0.f, 0.0f };
		Vector3D rotation = { 0.f, 0.f, 0.0f };
		Vector3D scale = { 1.f, 1.f, 1.f };
		Quaternion rot;


		Transform();
		Transform(Vector3D& pos);
		Transform(Vector3D& pos, Vector3D& rot);
		Transform(Vector3D pos, Quaternion rot);
		Transform(Vector3D& pos, Vector3D& rot, Vector3D& sca);

		~Transform();
	};
}






// Matrix are reads by their raws
class Matrix4D
{
	// 00 04
	// 01
	// 02
	// 03
private:

public:
	//Properties
	union
	{
		float e[16];
		float coord[4][4];
	};


	// Construction
	Matrix4D();
	Matrix4D(float tab[4][4]);
	Matrix4D(float tab[16]);
	~Matrix4D();

	Matrix4D operator+(const Matrix4D m2);
	Matrix4D operator-(const Matrix4D m2);
	Matrix4D operator*(const Matrix4D m2);
	Vector4D operator*(Vector4D v2);
	Vector3D operator*(Vector3D _v2);
	float operator[](const unsigned int index);

	void ToString(const float* coord, const char* name);

	static Matrix4D GetMatrixFromQuaternion(Quaternion quat);
	static Quaternion GetQuaternionFromMatrix(Matrix4D mat);

	void LoadIdentity();


	static Matrix4D TransposeMatrix(Matrix4D matrix);
	static Matrix4D InverseMatrix(Matrix4D matrix);
	static float Determinant2x2(Matrix4D matrix);
	static float Determinant3x3(Matrix4D matrix);
	static float Determinant4x4(Matrix4D matrix);
	static Matrix4D MatrixCofacteur(Matrix4D mat4x4);

	static int MatrixSignElement(int index);
	static Matrix4D Mat3x3ToMat2x2(int row, int columns, Matrix4D mat3x3);
	static Matrix4D Mat4x4ToMat3x3(int row, int columns, Matrix4D mat4x4);

	static Matrix4D CreateTransformMatrix(Physics::Transform& trans);
	static Matrix4D CreateTransformMatrix(const Vector3D position, const Quaternion rotation, const Vector3D scale = Vector3D(1,1,1));
	static Matrix4D CreateTranslationMatrix(const Vector3D& translation);
	static Matrix4D CreateScaleMatrix(const Vector3D& scale);
	static Matrix4D CreateXRotationMatrix(float angle);
	static Matrix4D CreateYRotationMatrix(float angle);
	static Matrix4D CreateZRotationMatrix(float angle);
	static Matrix4D CreateRotationMatrix(Vector3D angle);

	static Physics::Transform GetTransformFromMatrix(Matrix4D mat, Physics::Transform &transform);
	static Vector3D	GetPositionFromMatrix(Matrix4D mat);
	static Vector3D	GetRotationFromMatrix(Matrix4D mat);
	static Vector3D GetScaleFromMatrix(Matrix4D mat);



};

class Matrix3D
{
public:
	union
	{
		float e[9];
		float coord[3][3];
	};
	Matrix3D();
	Matrix3D(Matrix4D mat);
	~Matrix3D();

	void ToString(const float* coord, const char* name);
private:

};




Vector2D GetDirection2D(Vector2D posA, Vector2D posB);
float DotProduct2D(Vector2D a, Vector2D b);
float GetDeterminant2D(Vector2D a, Vector2D b);

float DotProduct(Vector3D a, Vector3D b);

// ----------- Rotation Vector 2D ---------

// Get angle of degree from a vector to a another.
// The angle is between 0 and 180 degree
float GetDegreAngle(Vector2D a, Vector2D b);

// Get angle of degree from a vector to a another.
// The angle is between -180 and 180 degree
float GetSignedDegreAngle(Vector2D a, Vector2D b);

// Rotate a vector
void RotateVector2D(Vector2D* direction, float angle);

// --------- Math algebric function ------------------

// Make transition between the start value and the end value
float LerpFloat(float start, float end, float t);

template <typename T>
T Lerp(T start, T end, float t);

Vector2D LerpVector(Vector2D start, Vector2D end, float t);

Quaternion Slerp(Quaternion start, Quaternion end, float t);

//Limit the value of a variable between a max and a min
float ClampFloat(float number, float min, float max);

int ClampInt(int number, int min, int max);

// --------- Random function --------
int32_t GetSeed();

int32_t getRandomNumber(int32_t state);

// Get a random number between a min (inclusive) and a max (non-inclusive)
int RandomNumber(int32_t& state, int min, int max);
float RandomNumber(int32_t& state, float min, float max);

// --------------- Vector 3D Functions --------------

// Multiplication of Vector3D
Vector3D CrossProduct(Vector3D a, Vector3D b);



// ---------- Matrix function ------------
void ExtractScaleFromMatrix(Matrix3D& mat);
void ExtractScaleFromMatrix(Matrix3D& mat, Vector3D& scale);
#include "mathLibrary.inl"

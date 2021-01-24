////////////////////////////////////////////////////////////////////
// Written by Isaac Sauer
// Class 2DAE08
// Year 2020-2021
////////////////////////////////////////////////////////////////////

#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <set>

//source: https://devblogs.microsoft.com/oldnewthing/20190711-00/?p=102682
#pragma region call_if_defined
//template<typename, typename = void>
//constexpr bool is_type_complete_v = false;
//
//template<typename T>
//constexpr bool is_type_complete_v
//<T, std::void_t<decltype(sizeof(T))>> = true;
//
//template<typename... T, typename TLambda>
//void call_if_defined(TLambda&& lambda)
//{
//	if constexpr ((... && is_type_complete_v<T>)) {
//		lambda(static_cast<T*>(nullptr)...);
//	}
//}
#pragma endregion call_if_defined

namespace Binary
{
	//based on https://stackoverflow.com/a/16824239
#pragma region has_<func>
	//check if object has a write function
	template<typename, typename T>
	struct has_Write {
		static_assert(
			std::integral_constant<T, false>::value,
			"Second template parameter needs to be of function type.");
	};
	template<typename C, typename Ret, typename... Args>
	struct has_Write<C, Ret(Args...)> {
	private:
		template<typename T>
		static constexpr auto check(T*)
			-> typename
			std::is_same<
			decltype(std::declval<T>().Write(std::declval<Args>()...)),
			Ret    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			>::type;  // attempt to call it and see if the return type is correct

		template<typename>
		static constexpr std::false_type check(...);

		typedef decltype(check<C>(0)) type;

	public:
		static constexpr bool value = type::value;
	};

	//based on https://stackoverflow.com/a/16824239
	//check if object has a read function
	template<typename, typename T>
	struct has_Read {
		static_assert(
			std::integral_constant<T, false>::value,
			"Second template parameter needs to be of function type.");
	};
	template<typename C, typename Ret, typename... Args>
	struct has_Read<C, Ret(Args...)> {
	private:
		template<typename T>
		static constexpr auto check(T*)
			-> typename
			std::is_same<
			decltype(std::declval<T>().Read(std::declval<Args>()...)),
			Ret    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			>::type;  // attempt to call it and see if the return type is correct

		template<typename>
		static constexpr std::false_type check(...);

		typedef decltype(check<C>(0)) type;

	public:
		static constexpr bool value = type::value;
	};
#pragma endregion has_<func>
	
	//safe/load variable to file as binary
	template<typename T>
	bool SaveToFile(const std::string& path, const T& var);
	template<typename T>
	bool LoadFromFile(const std::string& path, T& var);

	//safe/load variable to file as binary
	template<typename T>
	bool SaveSTLToFile(const std::string& path, const T& var);
	template<typename T>
	bool LoadSTLFromFile(const std::string& path, T& var);

	template<typename T>
	void Write(std::ofstream& out, const T& s);
	template<typename T>
	void Read(std::ifstream& in, T& s);

	template<typename T>
	void WriteOwn(std::ofstream& out, const T& v);
	template<typename T>
	void ReadOwn(std::ifstream& in, T& v);

	//void Write(std::ofstream& out) const {}
	//void Read(std::ifstream& in) {}

	struct Writers
	{
		template<typename T>
		static void WriteNonPOD(std::ofstream& out, const T& v);
		template<typename T>
		static void WritePOD(std::ofstream& out, const T& s);

		//std::string
		static void Write(std::ofstream& out, const std::string& s);

		//std::vector
		template<typename T>
		static void Write(std::ofstream& out, const std::vector<T>& vector);

		//std::unordered_map (A == std::string, B == self defined)
		template<typename A,typename B>
		static void Write(std::ofstream& out, const std::unordered_map<A, B>& umap);

		//std::set (T == self defined)
		template<typename T>
		static void Write(std::ofstream& out, const std::set<T>& set);
		
		//fallback
		static void Write(std::ofstream& out, ...) {};
	};
	struct Readers
	{
		template<typename T>
		static void ReadNonPOD(std::ifstream& in, T& s);
		template<typename T>
		static void ReadPOD(std::ifstream& in, T& s);

		//std::string
		static void Read(std::ifstream& in, std::string& s);

		//std::vector
		template<typename T>
		static void Read(std::ifstream& in, std::vector<T>& vector);

		//std::unordered_map (A == std::string, B == self defined)
		template<typename A, typename B>
		static void Read(std::ifstream& in, std::unordered_map<A, B>& umap);

		//std::set (T == self defined)
		template<typename T>
		static void Read(std::ifstream& in, std::set<T>& set);

		//fallback
		static void Read(std::ifstream& in, ...) {};
	};
}

#pragma region FileStream
template<typename T>
bool Binary::SaveToFile(const std::string& path, const T& var)
{
	std::ofstream out;
	out.open(path, std::ios::out | std::ios::binary);
	if (out.is_open())
	{
		Write(out, var);
		out.close();

		return true;
	}
	return false;
}
template<typename T>
bool Binary::LoadFromFile(const std::string& path, T& var)
{
	std::ifstream in;
	in.open(path, std::ios::in | std::ios::binary);
	if (in.is_open())
	{
		Read(in, var);
		in.close();

		return true;
	}
	return false;
}

template<typename T>
bool Binary::SaveSTLToFile(const std::string& path, const T& var)
{
	std::ofstream out;
	out.open(path, std::ios::out | std::ios::binary);
	if (out.is_open())
	{
		Writers::Write(out, var);
		out.close();

		return true;
	}
	return false;
}
template<typename T>
bool Binary::LoadSTLFromFile(const std::string& path, T& var)
{
	std::ifstream in;
	in.open(path, std::ios::in | std::ios::binary);
	if (in.is_open())
	{
		Readers::Read(in, var);
		in.close();

		return true;
	}
	return false;
}
#pragma endregion Functions

#pragma region InHouse
//std::vector
template<typename T>
void Binary::Writers::Write(std::ofstream& out, const std::vector<T>& vector)
{
	size_t size = vector.size();
	out.write((char*)&size, sizeof(size_t));

	for (auto& elem : vector)
		Binary::Write(out, elem);
}
template<typename T>
void Binary::Readers::Read(std::ifstream& in, std::vector<T>& vector)
{
	using type = typename std::decay<decltype(*vector.begin())>::type;

	size_t size = vector.size();
	in.read((char*)&size, sizeof(size_t));

	vector.resize(size);
	for (size_t i = 0; i < size; ++i)
	{
		type elem;
		Binary::Read(in, elem);
		vector[i] = elem;
	}
}

//std::unordered_map
template<typename A, typename B>
 void Binary::Writers::Write(std::ofstream& out, const std::unordered_map<A, B>& umap)
{
	 size_t size = umap.size();
	 out.write((char*)&size, sizeof(size_t));

	 for (auto& elem : umap)
	 {
		 Binary::WriteOwn(out, elem.first);
		 Binary::Write(out, elem.second);
	 }
}
template<typename A, typename B>
void Binary::Readers::Read(std::ifstream& in, std::unordered_map<A, B>& umap)
{
	using typeFirst = typename std::decay<decltype(umap.begin()->first)>::type;
	using typeSec = typename std::decay<decltype(umap.begin()->second)>::type;

	size_t size = umap.size();
	in.read((char*)&size, sizeof(size_t));

	for (size_t i = 0; i < size; ++i)
	{
		typeFirst first{};
		typeSec sec{};

		Binary::ReadOwn(in, first);
		Binary::Read(in, sec);
		umap.insert_or_assign(first, sec);
	}
}

//std::set
template<typename T>
void Binary::Writers::Write(std::ofstream& out, const std::set<T>& set)
{
	size_t size = set.size();
	out.write((char*)&size, sizeof(size_t));

	for (auto& elem : set)
		Binary::Write(out, elem);
}
template<typename T>
void Binary::Readers::Read(std::ifstream& in, std::set<T>& set)
{
	using type = typename std::decay<decltype(*set.begin())>::type;

	size_t size = set.size();
	in.read((char*)&size, sizeof(size_t));

	for (size_t i = 0; i < size; ++i)
	{
		type elem;
		Binary::Read(in, elem);
		set.insert(elem);
	}
}

//std::string
inline void Binary::Writers::Write(std::ofstream& out, const std::string& s)
{
	size_t len{ s.size() };
	out.write((char*)&len, sizeof(size_t));
	out.write(s.c_str(), len);
}
inline void Binary::Readers::Read(std::ifstream& in, std::string& s)
{
	size_t len{};
	in.read((char*)&len, sizeof(size_t));
	char* temp = new char[len + 1];
	in.read(temp, len);
	temp[len] = '\0';
	s = temp;
	delete[] temp;
}

//pod
template<typename T>
void Binary::Writers::WritePOD(std::ofstream& out, const T& s)
{
	out.write((char*)&s, sizeof(T));
}
template<typename T>
void Binary::Readers::ReadPOD(std::ifstream& in, T& s)
{
	in.read((char*)&s, sizeof(T));
}
#pragma endregion Functions

#pragma region Selector
//non pod
template<typename T>
void Binary::Writers::WriteNonPOD(std::ofstream& out, const T& v)
{
	if (has_Write<T, void(std::ofstream&)>::value)
		v.Write(out);
	else if (has_Write<Writers, void(std::ofstream&, const T&)>::value)
		Writers::Write(out, v);
}
template<typename T>
void Binary::Readers::ReadNonPOD(std::ifstream& in, T& s)
{
	if (has_Read<T, void(std::ifstream&)>::value)
		s.Read(in);
	else if (has_Read<Readers, void(std::ifstream&, const T&)>::value)
		Readers::Read(in, s);
}

//Used with objects which have the Write function
template<typename T>
void Binary::Write(std::ofstream& out, const T& s)
{
	if (std::is_pod<T>::value)
		Writers::WritePOD(out, s);
	else
		Writers::WriteNonPOD(out, s);

	//if (std::is_pod<T>::value)
	//	Writers::WritePOD(out, s);
	//else if (has_Write<Writers, void(std::ofstream&, const T&)>::value)
	//	Writers::Write(out, s);
	//else if (has_Write<T, void(std::ofstream&)>::value)
	//	s.Write(out);

	//if (std::is_pod<T>::value)
	//	Writers::WritePOD(out, s);
	//else if (has_Write<T, void(std::ofstream&)>::value)
	//	call_if_defined<T>([&](auto* p)
	//		{
	//			using Write = std::decay_t<decltype(*p)>;
	//			if (has_Write<Write, void(std::ofstream&)>::value)
	//				s.Write(out);
	//		});
	//else if (has_Write<Writers, void(std::ofstream&, const T&)>::value)
	//	Writers::Write(out, s);

}
//Used with objects which have the Read function
template<typename T>
void Binary::Read(std::ifstream& in, T& s)
{
	if (std::is_pod<T>::value)
		Readers::ReadPOD(in, s);
	else
		Readers::ReadNonPOD(in, s);

	//if (std::is_pod<T>::value)
	//	Readers::ReadPOD(in, s);
	//else if (has_Read<T, void(std::ifstream&)>::value) 
	//	s.Read(in);
	//else if (has_Read<Readers, void(std::ifstream&, const T&)>::value)
	//	Readers::Read(in, s);

	//if (std::is_pod<T>::value)
	//	Readers::ReadPOD(in, s);
	//else if (has_Read<T, void(std::ifstream&)>::value)
	//	call_if_defined<T>([&](auto* p)
	//		{
	//			using Read = std::decay_t<decltype(*p)>;
	//			if (has_Read<Read, void(std::ifstream&)>::value)
	//				s.Read(in);
	//		});
	//else if (has_Read<Readers, void(std::ifstream&, const T&)>::value)
	//	Readers::Read(in, s);
}

//for inhouse supported functions (for example: string, vector,...) also works with PODs
template<typename T>
void Binary::WriteOwn(std::ofstream& out, const T& v)
{
	if (std::is_pod<T>::value)
		Writers::WritePOD(out, v);
	else
		Writers::Write(out, v);
}
//for inhouse supported functions (for example: string, vector,...) also works with PODs
template<typename T>
void Binary::ReadOwn(std::ifstream& in, T& v)
{
	if (std::is_pod<T>::value)
		Readers::ReadPOD(in, v);
	else
		Readers::Read(in, v);
}
#pragma endregion Functions

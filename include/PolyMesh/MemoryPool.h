#pragma once
#include <vector>
#include <unordered_set>

namespace acamcad
{
	template< typename T>
	class MemoryPool
	{
	private:
		size_t BLOCK_SIZE = 1024;

		std::vector<T*> blocks;
		std::vector<T*> freeAdresses;

	public:
		MemoryPool() : BLOCK_SIZE(1024) {}
		~MemoryPool() { Clear(); }

	public:
		T* Request();
		void Recycle(T* object);
		void Reserve(size_t n);
		// no ~T()
		void fastClear();
		void Clear();	//依次调用所有的析构函数，然后归还空间

	private:
		void newBlock();

	};

	template<typename T>
	T* MemoryPool<T>::Request()
	{
		if (freeAdresses.empty())
			newBlock();
		T* freeAdress = freeAdresses.back();
		freeAdresses.pop_back();
		return freeAdress;
	}

	template<typename T>
	void MemoryPool<T>::Recycle(T* object) 
	{
//		if constexpr (!std::is_trivially_destructible_v<T>)
//			object->~T();
		object->~T();
		freeAdresses.push_back(object);
	}

	template<typename T>
	void MemoryPool<T>::Reserve(size_t n)
	{
		size_t blockNum = n / BLOCK_SIZE + static_cast<size_t>(n % BLOCK_SIZE > 0);
		for (size_t i = blocks.size(); i < blockNum; i++)
			newBlock();
	}

	template<typename T>
	void MemoryPool<T>::fastClear() 
	{
		for (auto block : blocks) {
			free(block);
		}
		blocks.clear();
		freeAdresses.clear();
	}

	template<typename T>
	void MemoryPool<T>::Clear() 
	{
		std::unordered_set<T*> freeAdressesSet(freeAdresses.begin(), freeAdresses.end());
		for (auto block : blocks) {
			for (size_t i = 0; i < BLOCK_SIZE; i++) {
				T* adress = block + i;
				if (freeAdressesSet.find(adress) == freeAdressesSet.end())
					adress->~T();
			}
			free(block);
		}
		blocks.clear();
		freeAdresses.clear();
	}

	template<typename T>
	void MemoryPool<T>::newBlock() {
//		T* block = (T*)operator new(sizeof(T)* BLOCK_SIZE);

		T* block = reinterpret_cast<T*>( malloc(BLOCK_SIZE * sizeof(T)) );

		blocks.push_back(block);
		for (size_t i = 0; i < BLOCK_SIZE; i++)
			freeAdresses.push_back(block + i);
	}

}
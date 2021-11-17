#pragma once

#include <memory>
#include <mutex>

namespace Planner {

	/// @brief Singleton template.
	template <class T>
	class Singleton {
	public:
		/// @brief Provide global access to the only instance of this class.
		static T& Get()
		{
			if (!m_instance_ptr) {
				m_instance_ptr = std::unique_ptr<T>(new T());
			}
			return *m_instance_ptr;
		}

		/// @brief Initialize the only instance of the class.
		static void Init()
		{
			Get();
		}

		/// @brief Provide global access to release/delete the instance.
		static void Release()
		{
			if (m_instance_ptr) {
				m_instance_ptr.reset();
			}
		}

	protected:
		Singleton() { }
		~Singleton() = default;

		static std::unique_ptr<T> m_instance_ptr;

	private:
		Singleton(const Singleton&) = delete;
		Singleton& operator=(const Singleton&) = delete;
	};

	template <class T>
	std::unique_ptr<T> Singleton<T>::m_instance_ptr = nullptr;

	/// @brief Singleton template for classes whose constructor have arguments.
	template <class T>
	class SingletonInit {
	public:
		/// @brief Provide global access to the only instance of this class.
		static T& Get()
		{
			assert(m_instance_ptr);
			return *m_instance_ptr;
		}

		/// @brief Initialize the only instance of the class.
		template <typename... Args>
		static void Init(Args... args)
		{
			assert(!m_instance_ptr);
			m_instance_ptr = std::unique_ptr<T>(new T(std::forward<Args>(args)...));
		}

		/// @brief Provide global access to release/delete the instance.
		static void Release()
		{
			if (m_instance_ptr) {
				m_instance_ptr.reset();
			}
		}

	protected:
		SingletonInit() { }
		~SingletonInit() = default;

		static std::unique_ptr<T> m_instance_ptr;

	private:
		SingletonInit(const SingletonInit&) = delete;
		SingletonInit& operator=(const SingletonInit&) = delete;
	};

	template <class T>
	std::unique_ptr<T> SingletonInit<T>::m_instance_ptr = nullptr;

	/// @brief Thread safe singleton template.
	template <class T>
	class SingletonThreadSafe {
	public:
		/// @brief Provide global access to the only instance of this class.
		static T& Get()
		{
			// Prevents the lock-step required each time the instance is requested
			if (!m_instance_ptr) {
				// Lock is required here though, to prevent multiple threads
				// initialising multiple instances of the class when it turns out it
				// has not been initialised yet
				std::lock_guard<std::mutex> lock(m_constructed);
				// Check to see if a previous thread has already initialised an
				// instance in the time it took to acquire a lock.
				if (!m_instance_ptr) {
					m_instance_ptr = std::unique_ptr<T>(new T());
				}
			}
			return *m_instance_ptr;
		}

		/// @brief Provide global access to release/delete the instance.
		static void Release()
		{
			std::lock_guard<std::mutex> lock(m_constructed);
			if (m_instance_ptr) {
				m_instance_ptr.reset();
			}
		}

	protected:
		SingletonThreadSafe() { }
		~SingletonThreadSafe() = default;

		static std::unique_ptr<T> m_instance_ptr;
		static std::mutex m_constructed;

	private:
		SingletonThreadSafe(const SingletonThreadSafe&) = delete;
		SingletonThreadSafe& operator=(const SingletonThreadSafe&) = delete;
	};

	template <class T>
	std::mutex SingletonThreadSafe<T>::m_constructed;
	template <class T>
	std::unique_ptr<T> SingletonThreadSafe<T>::m_instance_ptr = nullptr;
}

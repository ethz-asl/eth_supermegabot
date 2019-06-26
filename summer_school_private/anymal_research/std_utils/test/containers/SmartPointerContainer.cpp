/*!
* @file     SmartPointerIterator.hpp
* @author   Dario Bellicoso
* @date     Jan 26, 2018
* @brief
*/

#include <gtest/gtest.h>

#include <std_utils/containers/SmartPointerIterator.hpp>
#include <std_utils/containers/SmartPointerContainer.hpp>
#include <std_utils/containers/ObserverPointer.hpp>
#include <memory>
#include <vector>


//! A simple base class wrapping an int.
class MyTypeBase {
 public:
  virtual bool isBase() { return true; }
};

//! A simple class wrapping an int.
class MyInt : public MyTypeBase {
 public:
  explicit MyInt(int x) : x_(x) { }

  bool isBase() override { return false; }

  int getInt() const { return x_; }
  void setInt(int x) { x_ = x; }

  friend std::ostream& operator<<(std::ostream& os, const MyInt& myInt) {
    os << myInt.x_;
    return os;
  }

 private:
  int x_;
};

//! A simple class wrapping a double.
class MyDouble : public MyTypeBase {
 public:
  explicit MyDouble(int x) : x_(x) { }

  bool isBase() override { return false; }

  double getDouble() const { return x_; }
  void setDouble(double x) { x_ = x; }

  friend std::ostream& operator<<(std::ostream& os, const MyDouble& myDouble) {
    os << myDouble.x_;
    return os;
  }

 private:
  double x_;
};


template<typename ElementType_ = MyInt>
class MyUniquePtrContainerA : public std_utils::VectorOfUniquePtrs<ElementType_> {
 public:
  using Base = std_utils::VectorOfUniquePtrs<ElementType_>;
  using ElementType = ElementType_;
  using ItemSmartPtr = typename Base::smart_pointer_type;

  MyUniquePtrContainerA() {
    this->items_.reserve(5);
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{1}));
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{2}));
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{3}));
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{4}));
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{5}));
  }

  void addItem(ItemSmartPtr&& item) {
    this->items_.emplace_back(std::move(item));
  }
};

template<typename ElementType_ = MyInt>
class MyUniquePtrContainerB : public std_utils::VectorOfUniquePtrs<ElementType_> {
 public:
  using Base = std_utils::VectorOfUniquePtrs<ElementType_>;
  using ElementType = ElementType_;
  using ItemSmartPtr = typename Base::smart_pointer_type;

  MyUniquePtrContainerB() {
    this->items_.reserve(5);
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{6}));
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{7}));
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{8}));
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{9}));
    this->items_.push_back(std::unique_ptr<ElementType>(new ElementType{10}));
  }

  void addItem(ItemSmartPtr&& item) {
    this->items_.emplace_back(std::move(item));
  }
};

template<typename ElementType_ = MyInt>
class MySharedPtrContainer : public std_utils::VectorOfSharedPtrs<ElementType_> {
 public:
  using ElementType = ElementType_;

  MySharedPtrContainer() {
    this->items_.reserve(5);
    this->items_.push_back(std::shared_ptr<ElementType>(new ElementType{1}));
    this->items_.push_back(std::shared_ptr<ElementType>(new ElementType{2}));
    this->items_.push_back(std::shared_ptr<ElementType>(new ElementType{3}));
    this->items_.push_back(std::shared_ptr<ElementType>(new ElementType{4}));
    this->items_.push_back(std::shared_ptr<ElementType>(new ElementType{5}));
  }
};

template<typename ElementType_ = MyInt>
class MyUniquePtrDictionary : public std_utils::UnorderedMapOfUniquePtrs<std::string, ElementType_> {
 public:
  using ElementType = ElementType_;

  MyUniquePtrDictionary() {
    this->items_.reserve(5);
    this->items_.insert(std::make_pair("1", std::unique_ptr<ElementType>(new ElementType{1})));
    this->items_.insert(std::make_pair("2", std::unique_ptr<ElementType>(new ElementType{2})));
    this->items_.insert(std::make_pair("3", std::unique_ptr<ElementType>(new ElementType{3})));
    this->items_.insert(std::make_pair("4", std::unique_ptr<ElementType>(new ElementType{4})));
    this->items_.insert(std::make_pair("5", std::unique_ptr<ElementType>(new ElementType{5})));
  }
};








TEST(SmartPointerIterator, iterableClass) {
  MyUniquePtrContainerA<> unique_iterable;

  EXPECT_TRUE(unique_iterable.size() == 5);

  // Try to get the elements of the container wrapped in iterable.
  auto counter = 0;
  for (auto elem : unique_iterable) {
    EXPECT_TRUE(elem->getInt() == ++counter);
  }

  // Try to set the elements of the container wrapped in iterable.
  for (auto elem : unique_iterable) {
    elem->setInt(2);
  }

  // Try to get the modified elements of the container wrapped in iterable.
  for (auto elem : unique_iterable) {
    EXPECT_TRUE(elem->getInt() == 2);
  }

  unique_iterable.clear();
  EXPECT_TRUE(unique_iterable.size() == 0);

  // Try to get the elements of the container wrapped in the const iterable.
  const MyUniquePtrContainerA<const MyInt> const_unique_iterable;

  EXPECT_TRUE(const_unique_iterable.size() == 5);

  counter = 0;
  for (auto elem : const_unique_iterable) {
    EXPECT_TRUE(elem->getInt() == ++counter);
  }

  // Test the interface for shared pointers.
  MySharedPtrContainer<> shared_iterable;

  EXPECT_TRUE(shared_iterable.size() == 5);

  counter = 0;
  for (auto elem : shared_iterable) {
    EXPECT_TRUE(elem->getInt() == ++counter);
  }
  for (auto elem : shared_iterable) {
    elem->setInt(2);
  }
  for (auto elem : shared_iterable) {
    EXPECT_TRUE(elem->getInt() == 2);
  }


  {
    unique_iterable.clear();
    EXPECT_TRUE(unique_iterable.size() == 0);

    unique_iterable.addItem(MyUniquePtrContainerA<>::ItemSmartPtr(new MyUniquePtrContainerA<>::ElementType{0}));
    EXPECT_TRUE(unique_iterable.size() == 1);

    for (auto elem : unique_iterable) {
      EXPECT_TRUE(elem->getInt() == 0);
    }
  }
}

TEST(SmartPointerIterator, dictionaryClass) {
  MyUniquePtrDictionary<> unique_iterable;

  EXPECT_EQ(unique_iterable.size(), 5);

  // Try to get the elements of the container wrapped in iterable.
  for (auto elem : unique_iterable) {
    EXPECT_EQ(std::to_string(elem.second->getInt()), elem.first);
  }
}


TEST(SmartPointerIterator, observerPointer) {
  std::unique_ptr<MyInt> my_unique_int(new MyInt{2});
  {
    std_utils::observer_ptr<MyInt> my_observer_int(my_unique_int);
    EXPECT_TRUE(my_observer_int.get()->getInt() == 2);
  }

  // Try the observer pointer to the same object once more.
  {
    std_utils::observer_ptr<MyInt> my_observer_int(my_unique_int);
    EXPECT_TRUE(my_observer_int.get()->getInt() == 2);
  }

  MyUniquePtrContainerA<> unique_iterable;
  {
    auto observer_iterable = unique_iterable.make_observer_container();
    auto counter = 0;
    for (auto elem : observer_iterable) {
      EXPECT_TRUE(elem->getInt() == ++counter);
    }
  }
  {
    auto observer_iterable = unique_iterable.make_observer_container_of_type<MyTypeBase>();
    for (auto elem : observer_iterable) {
      EXPECT_FALSE( (*elem).isBase());
    }
  }
  {
    // Check if the contents of the original container are still valid.
    auto counter = 0;
    for (auto elem : unique_iterable) {
      EXPECT_TRUE(elem->getInt() == ++counter);
    }
  }

  {
    // Test inserting elements into a vector of observer pointers.
    std_utils::VectorOfObserverPtrs<MyTypeBase> base_observer_container;
    MyUniquePtrContainerA<MyInt> unique_iterable_a;
    MyUniquePtrContainerB<MyDouble> unique_iterable_b;
    auto observer_iterable_a = unique_iterable_a.make_observer_container_of_type<MyTypeBase>();
    auto observer_iterable_b = unique_iterable_b.make_observer_container_of_type<MyTypeBase>();
    base_observer_container.insert(base_observer_container.end(),
                                   observer_iterable_a.s_begin(),
                                   observer_iterable_a.s_end());

    base_observer_container.insert(base_observer_container.end(),
                                   observer_iterable_b.s_begin(),
                                   observer_iterable_b.s_end());

    // Elements in base_observer_containers should dynamically be of type MyInt.
    for (auto elem : base_observer_container) {
      EXPECT_FALSE( elem->isBase() );
    }

    // Check size of the new container.
    EXPECT_EQ(base_observer_container.size(), unique_iterable_a.size() + unique_iterable_b.size());
  }

}



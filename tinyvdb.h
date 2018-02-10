//////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018 Syoyo Fujita
// Copyright (c) 2012-2018 DreamWorks Animation LLC
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of Syoyo Fujita and DreamWorks Animation nor the names
// of its contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////
#ifndef TINY_VDB_H_
#define TINY_VDB_H_

#include <cassert>
#include <cstring>
#include <string>
#include <vector>

namespace tinyvdb {

#if __cplusplus > 199711L
// C++11
typedef uint64_t tinvdb_uint64;
typedef int64_t tinyvdb_int64;
#else
// Although `long long` is not a standard type pre C++11, assume it is defined
// as a compiler's extension.
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc++11-long-long"
#endif
typedef unsigned long long tinyvdb_uint64;
typedef long long tinyvdb_int64;
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#endif

// For OpenVDB code compatibility
typedef unsigned int Index32;
typedef tinyvdb_uint64 Index64;
typedef int Index;
typedef unsigned char Byte;
typedef short Int16;
typedef int Int32;
typedef tinyvdb_int64 Int64;
typedef double Real;

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpadded"
#endif

#if 0
template<int Index, int Log2Dim>
class NodeMask
{
 public:
  NodeMask();

};
#else

/// Return the number of on bits in the given 8-bit value.
inline Index32 CountOn(Byte v) {
// Simple LUT:
#ifndef _MSC_VER  // Visual C++ doesn't guarantee thread-safe initialization of
                  // local statics
  static
#endif
      /// @todo Move this table and others into, say, Util.cc
      const Byte numBits[256] = {
#define COUNTONB2(n) n, n + 1, n + 1, n + 2
#define COUNTONB4(n) \
  COUNTONB2(n), COUNTONB2(n + 1), COUNTONB2(n + 1), COUNTONB2(n + 2)
#define COUNTONB6(n) \
  COUNTONB4(n), COUNTONB4(n + 1), COUNTONB4(n + 1), COUNTONB4(n + 2)
          COUNTONB6(0), COUNTONB6(1), COUNTONB6(1), COUNTONB6(2)};
  return numBits[v];
#undef COUNTONB6
#undef COUNTONB4
#undef COUNTONB2

  // Sequentially clear least significant bits
  // Index32 c;
  // for (c = 0; v; c++)  v &= v - 0x01U;
  // return c;

  // This version is only fast on CPUs with fast "%" and "*" operations
  // return (v * UINT64_C(0x200040008001) & UINT64_C(0x111111111111111)) % 0xF;
}

/// Return the number of off bits in the given 8-bit value.
inline Index32 CountOff(Byte v) { return CountOn(static_cast<Byte>(~v)); }

/// Return the number of on bits in the given 32-bit value.
inline Index32 CountOn(Index32 v) {
  v = v - ((v >> 1) & 0x55555555U);
  v = (v & 0x33333333U) + ((v >> 2) & 0x33333333U);
  return (((v + (v >> 4)) & 0xF0F0F0FU) * 0x1010101U) >> 24;
}

/// Return the number of off bits in the given 32-bit value.
inline Index32 CountOff(Index32 v) { return CountOn(~v); }

/// Return the number of on bits in the given 64-bit value.
inline Index32 CountOn(Index64 v) {
  v = v - ((v >> 1) & 0x5555555555555555);
  v = (v & 0x3333333333333333) + ((v >> 2) & 0x3333333333333333);
  return static_cast<Index32>(
      (((v + (v >> 4)) & 0xF0F0F0F0F0F0F0F) * 0x101010101010101) >> 56);
}

/// Return the number of off bits in the given 64-bit value.
inline Index32 CountOff(Index64 v) { return CountOn(~v); }

/// Return the least significant on bit of the given 8-bit value.
inline Index32 FindLowestOn(Byte v) {
  assert(v);
#ifndef _MSC_VER  // Visual C++ doesn't guarantee thread-safe initialization of
                  // local statics
  static
#endif
      const Byte DeBruijn[8] = {0, 1, 6, 2, 7, 5, 4, 3};
  return DeBruijn[Byte((v & -v) * 0x1DU) >> 5];
}

/// Return the least significant on bit of the given 32-bit value.
inline Index32 FindLowestOn(Index32 v) {
  assert(v);
  // return ffs(v);
#ifndef _MSC_VER  // Visual C++ doesn't guarantee thread-safe initialization of
                  // local statics
  static
#endif
      const Byte DeBruijn[32] = {0,  1,  28, 2,  29, 14, 24, 3,  30, 22, 20,
                                 15, 25, 17, 4,  8,  31, 27, 13, 23, 21, 19,
                                 16, 7,  26, 12, 18, 6,  11, 5,  10, 9};
  return DeBruijn[Index32((v & -v) * 0x077CB531U) >> 27];
}

/// Return the least significant on bit of the given 64-bit value.
inline Index32 FindLowestOn(Index64 v) {
  assert(v);
  // return ffsll(v);
#ifndef _MSC_VER  // Visual C++ doesn't guarantee thread-safe initialization of
                  // local statics
  static
#endif
      const Byte DeBruijn[64] = {
          0,  1,  2,  53, 3,  7,  54, 27, 4,  38, 41, 8,  34, 55, 48, 28,
          62, 5,  39, 46, 44, 42, 22, 9,  24, 35, 59, 56, 49, 18, 29, 11,
          63, 52, 6,  26, 37, 40, 33, 47, 61, 45, 43, 21, 23, 58, 17, 10,
          51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15, 30, 14, 13, 12,
      };
  return DeBruijn[Index64((v & -v) * 0x022FDD63CC95386D) >> 58];
}

/// Return the most significant on bit of the given 32-bit value.
inline Index32 FindHighestOn(Index32 v) {
#ifndef _MSC_VER  // Visual C++ doesn't guarantee thread-safe initialization of
                  // local statics
  static
#endif
      const Byte DeBruijn[32] = {0,  9,  1,  10, 13, 21, 2,  29, 11, 14, 16,
                                 18, 22, 25, 3,  30, 8,  12, 20, 28, 15, 17,
                                 24, 7,  19, 27, 23, 6,  26, 5,  4,  31};
  v |= v >> 1;  // first round down to one less than a power of 2
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return DeBruijn[Index32(v * 0x07C4ACDDU) >> 27];
}

////////////////////////////////////////

/// Base class for the bit mask iterators
template <typename NodeMask>
class BaseMaskIterator {
 protected:
  Index32 mPos;             // bit position
  const NodeMask* mParent;  // this iterator can't change the parent_mask!

 public:
  BaseMaskIterator() : mPos(NodeMask::SIZE), mParent(NULL) {}
  // BaseMaskIterator(const BaseMaskIterator&) = default;
  BaseMaskIterator(Index32 pos, const NodeMask* parent)
      : mPos(pos), mParent(parent) {
    assert((parent == NULL && pos == 0) ||
           (parent != NULL && pos <= NodeMask::SIZE));
  }
  bool operator==(const BaseMaskIterator& iter) const {
    return mPos == iter.mPos;
  }
  bool operator!=(const BaseMaskIterator& iter) const {
    return mPos != iter.mPos;
  }
  bool operator<(const BaseMaskIterator& iter) const {
    return mPos < iter.mPos;
  }
  BaseMaskIterator& operator=(const BaseMaskIterator& iter) {
    mPos = iter.mPos;
    mParent = iter.mParent;
    return *this;
  }
  Index32 offset() const { return mPos; }
  Index32 pos() const { return mPos; }
  bool test() const {
    assert(mPos <= NodeMask::SIZE);
    return (mPos != NodeMask::SIZE);
  }
  operator bool() const { return this->test(); }
};  // class BaseMaskIterator

#if 0

/// @note This happens to be a const-iterator!
template <typename NodeMask>
class OnMaskIterator: public BaseMaskIterator<NodeMask>
{
private:
    typedef BaseMaskIterator<NodeMask> BaseType;
    //using BaseType = BaseMaskIterator<NodeMask>;
    //using BaseType::mPos;//bit position;
    //using BaseType::mParent;//this iterator can't change the parent_mask!
public:
    OnMaskIterator() : BaseType() {}
    OnMaskIterator(Index32 pos,const NodeMask *parent) : BaseType(pos,parent) {}
    void increment()
    {
        assert(mParent != NULL);
        mPos = mParent->findNextOn(mPos+1);
        assert(mPos <= NodeMask::SIZE);
    }
    void increment(Index n) { while(n-- && this->next()) ; }
    bool next()
    {
        this->increment();
        return this->test();
    }
    bool operator*() const {return true;}
    OnMaskIterator& operator++()
    {
        this->increment();
        return *this;
    }
}; // class OnMaskIterator


template <typename NodeMask>
class OffMaskIterator: public BaseMaskIterator<NodeMask>
{
private:
    using BaseType = BaseMaskIterator<NodeMask>;
    using BaseType::mPos;//bit position;
    using BaseType::mParent;//this iterator can't change the parent_mask!
public:
    OffMaskIterator() : BaseType()  {}
    OffMaskIterator(Index32 pos,const NodeMask *parent) : BaseType(pos,parent) {}
    void increment()
    {
        assert(mParent != NULL);
        mPos=mParent->findNextOff(mPos+1);
        assert(mPos <= NodeMask::SIZE);
    }
    void increment(Index n) { while(n-- && this->next()) ; }
    bool next()
    {
        this->increment();
        return this->test();
    }
    bool operator*() const {return false;}
    OffMaskIterator& operator++()
    {
        this->increment();
        return *this;
    }
}; // class OffMaskIterator


template <typename NodeMask>
class DenseMaskIterator: public BaseMaskIterator<NodeMask>
{
private:
    using BaseType = BaseMaskIterator<NodeMask>;
    using BaseType::mPos;//bit position;
    using BaseType::mParent;//this iterator can't change the parent_mask!

public:
    DenseMaskIterator() : BaseType() {}
    DenseMaskIterator(Index32 pos,const NodeMask *parent) : BaseType(pos,parent) {}
    void increment()
    {
        assert(mParent != NULL);
        mPos += 1;//careful - the increment might go beyond the end
        assert(mPos<= NodeMask::SIZE);
    }
    void increment(Index n) { while(n-- && this->next()) ; }
    bool next()
    {
        this->increment();
        return this->test();
    }
    bool operator*() const {return mParent->isOn(mPos);}
    DenseMaskIterator& operator++()
    {
        this->increment();
        return *this;
    }
}; // class DenseMaskIterator
#endif

/// @brief Bit mask for the internal and leaf nodes of VDB. This
/// is a 64-bit implementation.
///
/// @note A template specialization for Log2Dim=1 and Log2Dim=2 are
/// given below.
// template <Index Log2Dim>
class NodeMask {
 public:
  // static_assert(Log2Dim > 2, "expected NodeMask template specialization, got
  // base template");
  Index32 LOG2DIM;
  Index32 DIM;
  Index32 SIZE;
  Index32 WORD_COUNT;

  // static const Index32 LOG2DIM = Log2Dim;
  // static const Index32 DIM = 1 << Log2Dim;
  // static const Index32 SIZE = 1 << 3 * Log2Dim;
  // static const Index32 WORD_COUNT = SIZE >> 6;  // 2^6=64
  // using Word = Index64;
  typedef Index64 Word;

 private:
  // The bits are represented as a linear array of Words, and the
  // size of a Word is 32 or 64 bits depending on the platform.
  // The BIT_MASK is defined as the number of bits in a Word - 1
  // static const Index32 BIT_MASK   = sizeof(void*) == 8 ? 63 : 31;
  // static const Index32 LOG2WORD   = BIT_MASK == 63 ? 6 : 5;
  // static const Index32 WORD_COUNT = SIZE >> LOG2WORD;
  // using Word = boost::mpl::if_c<BIT_MASK == 63, Index64, Index32>::type;

  std::vector<Word> mWords;  // only member data!

 public:
  /// Default constructor sets all bits off
  NodeMask(Index32 log2dim) {
    LOG2DIM = log2dim;
    DIM = 1 << log2dim;
    SIZE = 1 << 3 * log2dim;
    WORD_COUNT = SIZE >> 6;  // 2^6=64

    mWords.resize(WORD_COUNT);

    this->setOff();
  }
  /// All bits are set to the specified state
  NodeMask(Index32 log2dim, bool on) {
    LOG2DIM = log2dim;
    DIM = 1 << log2dim;
    SIZE = 1 << 3 * log2dim;
    WORD_COUNT = SIZE >> 6;  // 2^6=64

    this->set(on);
  }
  /// Copy constructor
  NodeMask(const NodeMask& other) { *this = other; }
  /// Destructor
  ~NodeMask() {}
  /// Assignment operator
  NodeMask& operator=(const NodeMask& other) {
    mWords = other.mWords;
    return *this;
    // Index32 n = WORD_COUNT;
    // const Word* w2 = other.mWords;
    // for (Word *w1 = mWords; n--; ++w1, ++w2) *w1 = *w2;
  }

#if 0
    using OnIterator = OnMaskIterator<NodeMask>;
    using OffIterator = OffMaskIterator<NodeMask>;
    using DenseIterator = DenseMaskIterator<NodeMask>;

    OnIterator beginOn() const       { return OnIterator(this->findFirstOn(),this); }
    OnIterator endOn() const         { return OnIterator(SIZE,this); }
    OffIterator beginOff() const     { return OffIterator(this->findFirstOff(),this); }
    OffIterator endOff() const       { return OffIterator(SIZE,this); }
    DenseIterator beginDense() const { return DenseIterator(0,this); }
    DenseIterator endDense() const   { return DenseIterator(SIZE,this); }
#endif

  bool operator==(const NodeMask& other) const {
    int n = int(WORD_COUNT);
    for (const Word *w1 = mWords.data(), *w2 = other.mWords.data();
         n-- && *w1++ == *w2++;)
      ;
    return n == -1;
  }

  bool operator!=(const NodeMask& other) const { return !(*this == other); }

  //
  // Bitwise logical operations
  //

  /// @brief Apply a functor to the words of the this and the other mask.
  ///
  /// @details An example that implements the "operator&=" method:
  /// @code
  /// struct Op { inline void operator()(W &w1, const W& w2) const { w1 &= w2; }
  /// };
  /// @endcode
  template <typename WordOp>
  const NodeMask& foreach (const NodeMask& other, const WordOp& op) {
    Word* w1 = mWords.data();
    const Word* w2 = other.mWords.data();
    for (Index32 n = WORD_COUNT; n--; ++w1, ++w2) op(*w1, *w2);
    return *this;
  }
  template <typename WordOp>
  const NodeMask& foreach (const NodeMask& other1, const NodeMask& other2,
                           const WordOp& op) {
    Word* w1 = mWords.data();
    const Word *w2 = other1.mWords.data(), *w3 = other2.mWords.data();
    for (Index32 n = WORD_COUNT; n--; ++w1, ++w2, ++w3) op(*w1, *w2, *w3);
    return *this;
  }
  template <typename WordOp>
  const NodeMask& foreach (const NodeMask& other1, const NodeMask& other2,
                           const NodeMask& other3, const WordOp& op) {
    Word* w1 = mWords.data();
    const Word *w2 = other1.mWords.data(), *w3 = other2.mWords.data(),
               *w4 = other3.mWords.data();
    for (Index32 n = WORD_COUNT; n--; ++w1, ++w2, ++w3, ++w4)
      op(*w1, *w2, *w3, *w4);
    return *this;
  }
  /// @brief Bitwise intersection
  const NodeMask& operator&=(const NodeMask& other) {
    Word* w1 = mWords.data();
    const Word* w2 = other.mWords.data();
    for (Index32 n = WORD_COUNT; n--; ++w1, ++w2) *w1 &= *w2;
    return *this;
  }
  /// @brief Bitwise union
  const NodeMask& operator|=(const NodeMask& other) {
    Word* w1 = mWords.data();
    const Word* w2 = other.mWords.data();
    for (Index32 n = WORD_COUNT; n--; ++w1, ++w2) *w1 |= *w2;
    return *this;
  }
  /// @brief Bitwise difference
  const NodeMask& operator-=(const NodeMask& other) {
    Word* w1 = mWords.data();
    const Word* w2 = other.mWords.data();
    for (Index32 n = WORD_COUNT; n--; ++w1, ++w2) *w1 &= ~*w2;
    return *this;
  }
  /// @brief Bitwise XOR
  const NodeMask& operator^=(const NodeMask& other) {
    Word* w1 = mWords.data();
    const Word* w2 = other.mWords.data();
    for (Index32 n = WORD_COUNT; n--; ++w1, ++w2) *w1 ^= *w2;
    return *this;
  }
  NodeMask operator!() const {
    NodeMask m(*this);
    m.toggle();
    return m;
  }
  NodeMask operator&(const NodeMask& other) const {
    NodeMask m(*this);
    m &= other;
    return m;
  }
  NodeMask operator|(const NodeMask& other) const {
    NodeMask m(*this);
    m |= other;
    return m;
  }
  NodeMask operator^(const NodeMask& other) const {
    NodeMask m(*this);
    m ^= other;
    return m;
  }

  /// Return the byte size of this NodeMask
  Index32 memUsage() const {
    return static_cast<Index32>(WORD_COUNT * sizeof(Word));
  }
  /// Return the total number of on bits
  Index32 countOn() const {
    Index32 sum = 0, n = WORD_COUNT;
    for (const Word* w = mWords.data(); n--; ++w) sum += CountOn(*w);
    return sum;
  }
  /// Return the total number of on bits
  Index32 countOff() const { return SIZE - this->countOn(); }
  /// Set the <i>n</i>th  bit on
  void setOn(Index32 n) {
    assert((n >> 6) < WORD_COUNT);
    mWords[n >> 6] |= Word(1) << (n & 63);
  }
  /// Set the <i>n</i>th bit off
  void setOff(Index32 n) {
    assert((n >> 6) < WORD_COUNT);
    mWords[n >> 6] &= ~(Word(1) << (n & 63));
  }
  /// Set the <i>n</i>th bit to the specified state
  void set(Index32 n, bool On) { On ? this->setOn(n) : this->setOff(n); }
  /// Set all bits to the specified state
  void set(bool on) {
    const Word state = on ? ~Word(0) : Word(0);
    Index32 n = WORD_COUNT;
    for (Word* w = mWords.data(); n--; ++w) *w = state;
  }
  /// Set all bits on
  void setOn() {
    Index32 n = WORD_COUNT;
    for (Word* w = mWords.data(); n--; ++w) *w = ~Word(0);
  }
  /// Set all bits off
  void setOff() {
    Index32 n = WORD_COUNT;
    for (Word* w = mWords.data(); n--; ++w) *w = Word(0);
  }
  /// Toggle the state of the <i>n</i>th bit
  void toggle(Index32 n) {
    assert((n >> 6) < WORD_COUNT);
    mWords[n >> 6] ^= Word(1) << (n & 63);
  }
  /// Toggle the state of all bits in the mask
  void toggle() {
    Index32 n = WORD_COUNT;
    for (Word* w = mWords.data(); n--; ++w) *w = ~*w;
  }
  /// Set the first bit on
  void setFirstOn() { this->setOn(0); }
  /// Set the last bit on
  void setLastOn() { this->setOn(SIZE - 1); }
  /// Set the first bit off
  void setFirstOff() { this->setOff(0); }
  /// Set the last bit off
  void setLastOff() { this->setOff(SIZE - 1); }
  /// Return @c true if the <i>n</i>th bit is on
  bool isOn(Index32 n) const {
    assert((n >> 6) < WORD_COUNT);
    return 0 != (mWords[n >> 6] & (Word(1) << (n & 63)));
  }
  /// Return @c true if the <i>n</i>th bit is off
  bool isOff(Index32 n) const { return !this->isOn(n); }
  /// Return @c true if all the bits are on
  bool isOn() const {
    int n = int(WORD_COUNT);
    for (const Word* w = mWords.data(); n-- && *w++ == ~Word(0);)
      ;
    return n == -1;
  }
  /// Return @c true if all the bits are off
  bool isOff() const {
    int n = int(WORD_COUNT);
    for (const Word* w = mWords.data(); n-- && *w++ == Word(0);)
      ;
    return n == -1;
  }
  /// Return @c true if bits are either all off OR all on.
  /// @param isOn Takes on the values of all bits if the method
  /// returns true - else it is undefined.
  bool isConstant(bool& isOn) const {
    isOn = (mWords[0] == ~Word(0));  // first word has all bits on
    if (!isOn && mWords[0] != Word(0)) return false;  // early out
    const Word *w = mWords.data() + 1, *n = mWords.data() + WORD_COUNT;
    while (w < n && *w == mWords[0]) ++w;
    return w == n;
  }
  Index32 findFirstOn() const {
    Index32 n = 0;
    const Word* w = mWords.data();
    for (; n < WORD_COUNT && !*w; ++w, ++n)
      ;
    return n == WORD_COUNT ? SIZE : (n << 6) + FindLowestOn(*w);
  }
  Index32 findFirstOff() const {
    Index32 n = 0;
    const Word* w = mWords.data();
    for (; n < WORD_COUNT && !~*w; ++w, ++n)
      ;
    return n == WORD_COUNT ? SIZE : (n << 6) + FindLowestOn(~*w);
  }

  //@{
  /// Return the <i>n</i>th word of the bit mask, for a word of arbitrary size.
  template <typename WordT>
  WordT getWord(Index n) const {
    assert(n * 8 * sizeof(WordT) < SIZE);
    return reinterpret_cast<const WordT*>(mWords)[n];
  }
  template <typename WordT>
  WordT& getWord(Index n) {
    assert(n * 8 * sizeof(WordT) < SIZE);
    return reinterpret_cast<WordT*>(mWords)[n];
  }
  //@}

  void save(std::ostream& os) const {
    os.write(reinterpret_cast<const char*>(mWords.data()), this->memUsage());
  }
  void load(std::istream& is) {
    is.read(reinterpret_cast<char*>(mWords.data()), this->memUsage());
  }
  void seek(std::istream& is) const {
    is.seekg(this->memUsage(), std::ios_base::cur);
  }
  /// @brief simple print method for debugging
  void printInfo(std::ostream& os = std::cout) const {
    os << "NodeMask: Dim=" << DIM << " Log2Dim=" << LOG2DIM
       << " Bit count=" << SIZE << " word count=" << WORD_COUNT << std::endl;
  }
  void printBits(std::ostream& os = std::cout, Index32 max_out = 80u) const {
    const Index32 n = (SIZE > max_out ? max_out : SIZE);
    for (Index32 i = 0; i < n; ++i) {
      if (!(i & 63))
        os << "||";
      else if (!(i % 8))
        os << "|";
      os << this->isOn(i);
    }
    os << "|" << std::endl;
  }
  void printAll(std::ostream& os = std::cout, Index32 max_out = 80u) const {
    this->printInfo(os);
    this->printBits(os, max_out);
  }

  Index32 findNextOn(Index32 start) const {
    Index32 n = start >> 6;            // initiate
    if (n >= WORD_COUNT) return SIZE;  // check for out of bounds
    Index32 m = start & 63;
    Word b = mWords[n];
    if (b & (Word(1) << m)) return start;          // simpel case: start is on
    b &= ~Word(0) << m;                            // mask out lower bits
    while (!b && ++n < WORD_COUNT) b = mWords[n];  // find next none-zero word
    return (!b ? SIZE : (n << 6) + FindLowestOn(b));  // catch last word=0
  }

  Index32 findNextOff(Index32 start) const {
    Index32 n = start >> 6;            // initiate
    if (n >= WORD_COUNT) return SIZE;  // check for out of bounds
    Index32 m = start & 63;
    Word b = ~mWords[n];
    if (b & (Word(1) << m)) return start;           // simpel case: start is on
    b &= ~Word(0) << m;                             // mask out lower bits
    while (!b && ++n < WORD_COUNT) b = ~mWords[n];  // find next none-zero word
    return (!b ? SIZE : (n << 6) + FindLowestOn(b));  // catch last word=0
  }
};  // NodeMask

#endif

class GridDescriptor {
 public:
  GridDescriptor();
  GridDescriptor(const std::string &name, const std::string &grid_type,
                 bool save_float_as_half = false);
  // GridDescriptor(const GridDescriptor &rhs);
  // GridDescriptor& operator=(const GridDescriptor &rhs);
  ~GridDescriptor();

  const std::string &GridName() const { return grid_name_; }

  bool IsInstance() const { return !instance_parent_name_.empty(); }

  bool SaveFloatAsHalf() const { return save_float_as_half_; }

  tinyvdb_int64 GridPos() const { return grid_pos_; }

  tinyvdb_int64 BlockPos() const { return block_pos_; }

  tinyvdb_int64 EndPos() const { return end_pos_; }

  static std::string AddSuffix(const std::string &name, int n);
  static std::string StripSuffix(const std::string &name);

  ///
  /// Read GridDescriptor from a stream.
  ///
  bool Read(std::ifstream &is, const unsigned int file_version,
            std::string *err);

 private:
  std::string grid_name_;
  std::string unique_name_;
  std::string instance_parent_name_;
  std::string grid_type_;

  bool save_float_as_half_;  // use fp16?
  tinyvdb_int64 grid_pos_;
  tinyvdb_int64 block_pos_;
  tinyvdb_int64 end_pos_;
};

typedef enum {
  NODE_TYPE_ROOT = 0,
  NODE_TYPE_INTERNAL = 1,
  NODE_TYPE_LEAF = 2
} NodeType;

typedef enum {
  VALUE_TYPE_NULL = 0,
  VALUE_TYPE_FLOAT = 1,
  VALUE_TYPE_HALF = 2,
  VALUE_TYPE_BOOL = 3,
  VALUE_TYPE_DOUBLE = 4,
  VALUE_TYPE_INT = 5
} ValueType;

static size_t GetValueTypeSize(const ValueType type) {
  if (type == VALUE_TYPE_FLOAT) {
    return sizeof(float);
  } else if (type == VALUE_TYPE_HALF) {
    return sizeof(short);
  } else if (type == VALUE_TYPE_BOOL) {
    return 1;
  } else if (type == VALUE_TYPE_DOUBLE) {
    return sizeof(double);
  }
  return 0;
}

// Simple class to represent value object
class Value {
 public:
  Value() : type_(VALUE_TYPE_NULL) {}

  explicit Value(bool b) : type_(VALUE_TYPE_BOOL) { boolean_value_ = b; }
  explicit Value(float f) : type_(VALUE_TYPE_FLOAT) { float_value_ = f; }
  explicit Value(double d) : type_(VALUE_TYPE_DOUBLE) { double_value_ = d; }
  explicit Value(int n) : type_(VALUE_TYPE_INT) { int_value_ = n; }

  ValueType Type() const { return type_; }

  bool IsBool() const { return (type_ == VALUE_TYPE_BOOL); }
  bool IsFloat() const { return (type_ == VALUE_TYPE_FLOAT); }
  bool IsDouble() const { return (type_ == VALUE_TYPE_DOUBLE); }
  bool IsInt() const { return (type_ == VALUE_TYPE_INT); }

  // Accessor
  template <typename T>
  const T &Get() const;
  template <typename T>
  T &Get();

  size_t Size() const {
    size_t len = 0;
    switch (type_) {
      case VALUE_TYPE_BOOL:
        len = 1;
        break;
      case VALUE_TYPE_HALF:
        len = sizeof(short);
        break;
      case VALUE_TYPE_INT:
        len = sizeof(int);
        break;
      case VALUE_TYPE_FLOAT:
        len = sizeof(float);
        break;
      case VALUE_TYPE_DOUBLE:
        len = sizeof(double);
        break;
      case VALUE_TYPE_NULL:
        len = 0;
        break;
    }

    return len;
  }

 protected:
  ValueType type_;

  int int_value_;
  float float_value_;
  double double_value_;
  bool boolean_value_;
};

#define TINYVDB_VALUE_GET(ctype, var)             \
  template <>                                     \
  inline const ctype &Value::Get<ctype>() const { \
    return var;                                   \
  }                                               \
  template <>                                     \
  inline ctype &Value::Get<ctype>() {             \
    return var;                                   \
  }
TINYVDB_VALUE_GET(bool, boolean_value_)
TINYVDB_VALUE_GET(double, double_value_)
TINYVDB_VALUE_GET(int, int_value_)
TINYVDB_VALUE_GET(float, float_value_)
#undef TINYVDB_VALUE_GET

static std::ostream &operator<<(std::ostream &os, const Value &value) {
  if (value.Type() == VALUE_TYPE_NULL) {
    os << "NULL";
  } else if (value.Type() == VALUE_TYPE_BOOL) {
    os << value.Get<bool>();
  } else if (value.Type() == VALUE_TYPE_FLOAT) {
    os << value.Get<float>();
  } else if (value.Type() == VALUE_TYPE_INT) {
    os << value.Get<int>();
  } else if (value.Type() == VALUE_TYPE_DOUBLE) {
    os << value.Get<double>();
  }

  return os;
}

class NodeInfo {
 public:
  NodeInfo(NodeType node_type, ValueType value_type, Index32 log2dim)
      : node_type_(node_type), value_type_(value_type), log2dim_(log2dim) {}

  NodeType node_type() const { return node_type_; }

  ValueType value_type() const { return value_type_; }

  Index32 log2dim() const { return log2dim_; }

 private:
  NodeType node_type_;
  ValueType value_type_;
  Index32 log2dim_;
};

class InternalNode;

// Assume `ValueT` is pod type(e.g. float).
template <typename ValueT, typename ChildT>
class NodeUnion {
 private:
  union {
    ChildT *mChild;
    ValueT mValue;
  };

 public:
  NodeUnion() : mChild(NULL) {}
  NodeUnion(const NodeUnion &other) : mChild(NULL) {
    std::memcpy(this, &other, sizeof(*this));
  }
  NodeUnion &operator=(const NodeUnion &rhs) {
    std::memcpy(this, &rhs, sizeof(*this));
    return *this;
  }

  ChildT *getChild() const { return mChild; }
  void setChild(ChildT *child) { mChild = child; }

  const ValueT &getValue() const { return mValue; }
  ValueT &getValue() { return mValue; }
  void setValue(const ValueT &val) { mValue = val; }
};

class Node {
 public:
  Node(NodeInfo node_info) : node_info_(node_info) {}
  virtual ~Node();

  virtual bool ReadTopology(std::istream &is, const bool fromHalf,
                            const unsigned int file_version) = 0;

 protected:
  NodeInfo node_info_;
};

Node::~Node() {}

class LeafNode : public Node {
 public:
  LeafNode(const NodeInfo node_info) : Node(node_info) {}
  ~LeafNode();

  bool ReadTopology(std::istream &is, const bool fromHalf,
                    const unsigned int file_version);

 private:
};

LeafNode::~LeafNode() {}

bool LeafNode::ReadTopology(std::istream &is, const bool fromHalf,
                            const unsigned int file_version) {
  (void)is;
  (void)fromHalf;
  (void)file_version;

  return false;
}

///
/// Internal node have `InternalNode` or `LeafNode` as children.
///
class InternalNode : public Node {
 public:
  // static const Index LOG2DIM = Log2Dim,  // log2 of tile count in one
  // dimension
  //    TOTAL = Log2Dim +
  //            ChildNodeType::TOTAL,  // log2 of voxel count in one dimension
  //    DIM = 1 << TOTAL,              // total voxel count in one dimension
  //    NUM_VALUES =
  //        1 << (3 * Log2Dim),  // total voxel count represented by this node
  //    LEVEL = 1 + ChildNodeType::LEVEL;  // level 0 = leaf
  // static const Index64 NUM_VOXELS =
  //    uint64_t(1) << (3 * TOTAL);  // total voxel count represented by this
  //    node
  // static const Index NUM_VALUES = 1 << (3 * Log2Dim); // total voxel count
  // represented by this node

  InternalNode(NodeInfo node_info, Node *child_node)
      : Node(node_info),
        child_node_(child_node),
        child_mask_(node_info.log2dim()),
        value_mask_(node_info.log2dim()) {
    origin_[0] = 0.0f;
    origin_[1] = 0.0f;
    origin_[2] = 0.0f;
  }
  ~InternalNode() {}

  bool ReadTopology(std::istream &is, const bool from_half,
                    const unsigned int file_version);

 private:
  Node *child_node_;
  // NodeUnion<ValueType, InternalNode> nodes_[NUM_VALUES];

  NodeMask child_mask_;
  NodeMask value_mask_;
  // NodeMask<Log2Dim> child_mask_;
  // NodeMask<Log2Dim> value_mask_;

  int origin_[3];
};

class RootNode : public Node {
 public:
  RootNode(const NodeInfo node_info, Node *child_node)
      : Node(node_info), child_node_(child_node) {}
  ~RootNode() {}

  bool ReadTopology(std::istream &is, const bool fromHalf,
                    const unsigned int file_version);

 private:
  Node *child_node_;
  Value background_;  // Background(region of un-interested area) value
};

///
/// Reader class for Tree data
///
class TreeReader {
 public:
  TreeReader();
  ~TreeReader();

  bool Read(std::istream &is);
};

#ifdef __clang__
#pragma clang diagnostic pop
#endif

///
/// Parse VDB header from a file.
/// Returns true upon success.
/// Returns false when failed to parse VDB header and store error message to
/// `err`.
///
bool ParseVDBHeader(const std::string &filename, std::string *err);

///
/// Parse VDB header from memory.
/// Returns true upon success.
/// Returns false when failed to parse VDB header and store error message to
/// `err`.
///
bool ParseVDBHeader(const unsigned char *data, const size_t len,
                    std::string *err);

///
/// Write VDB data to a file.
///
bool SaveVDB(const std::string &filename, std::string *err);

}  // namespace tinyvdb

#ifdef TINYVDB_IMPLEMENTATION

#include <cassert>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

namespace tinyvdb {

const int kOPENVDB_MAGIC = 0x56444220;

// HACK
#define MINIZ_LITTLE_ENDIAN (1)

///
/// TinyVDB file version.
///
const unsigned int kTINYVDB_FILE_VERSION = 222;

/// Notable file format version numbers
enum {
  OPENVDB_FILE_VERSION_ROOTNODE_MAP = 213,
  OPENVDB_FILE_VERSION_INTERNALNODE_COMPRESSION = 214,
  OPENVDB_FILE_VERSION_SIMPLIFIED_GRID_TYPENAME = 215,
  OPENVDB_FILE_VERSION_GRID_INSTANCING = 216,
  OPENVDB_FILE_VERSION_BOOL_LEAF_OPTIMIZATION = 217,
  OPENVDB_FILE_VERSION_BOOST_UUID = 218,
  OPENVDB_FILE_VERSION_NO_GRIDMAP = 219,
  OPENVDB_FILE_VERSION_NEW_TRANSFORM = 219,
  OPENVDB_FILE_VERSION_SELECTIVE_COMPRESSION = 220,
  OPENVDB_FILE_VERSION_FLOAT_FRUSTUM_BBOX = 221,
  OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION = 222,
  OPENVDB_FILE_VERSION_BLOSC_COMPRESSION = 223,
  OPENVDB_FILE_VERSION_POINT_INDEX_GRID = 223,
  OPENVDB_FILE_VERSION_MULTIPASS_IO = 224
};

enum {
  COMPRESS_NONE = 0,
  COMPRESS_ZIP = 0x1,
  COMPRESS_ACTIVE_MASK = 0x2,
  COMPRESS_BLOSC = 0x4
};

namespace {

// In order not to break backward compatibility with existing VDB files,
// grids stored using 16-bit half floats are flagged by adding the following
// suffix to the grid's type name on output.  The suffix is removed on input
// and the grid's "save float as half" flag set accordingly.
const char *HALF_FLOAT_TYPENAME_SUFFIX = "_HalfFloat";

const char *SEP = "\x1e";  // ASCII "record separator"

}  // namespace

// https://gist.github.com/rygorous/2156668
// Reuse MINIZ_LITTLE_ENDIAN flag from miniz.
union FP32 {
  unsigned int u;
  float f;
  struct {
#if MINIZ_LITTLE_ENDIAN
    unsigned int Mantissa : 23;
    unsigned int Exponent : 8;
    unsigned int Sign : 1;
#else
    unsigned int Sign : 1;
    unsigned int Exponent : 8;
    unsigned int Mantissa : 23;
#endif
  } s;
};

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpadded"
#endif

union FP16 {
  unsigned short u;
  struct {
#if MINIZ_LITTLE_ENDIAN
    unsigned int Mantissa : 10;
    unsigned int Exponent : 5;
    unsigned int Sign : 1;
#else
    unsigned int Sign : 1;
    unsigned int Exponent : 5;
    unsigned int Mantissa : 10;
#endif
  } s;
};

#ifdef __clang__
#pragma clang diagnostic pop
#endif

static inline FP32 half_to_float(FP16 h) {
  static const FP32 magic = {113 << 23};
  static const unsigned int shifted_exp = 0x7c00
                                          << 13;  // exponent mask after shift
  FP32 o;

  o.u = (h.u & 0x7fffU) << 13U;           // exponent/mantissa bits
  unsigned int exp_ = shifted_exp & o.u;  // just the exponent
  o.u += (127 - 15) << 23;                // exponent adjust

  // handle exponent special cases
  if (exp_ == shifted_exp)    // Inf/NaN?
    o.u += (128 - 16) << 23;  // extra exp adjust
  else if (exp_ == 0)         // Zero/Denormal?
  {
    o.u += 1 << 23;  // extra exp adjust
    o.f -= magic.f;  // renormalize
  }

  o.u |= (h.u & 0x8000U) << 16U;  // sign bit
  return o;
}

static inline FP16 float_to_half_full(FP32 f) {
  FP16 o = {0};

  // Based on ISPC reference code (with minor modifications)
  if (f.s.Exponent == 0)  // Signed zero/denormal (which will underflow)
    o.s.Exponent = 0;
  else if (f.s.Exponent == 255)  // Inf or NaN (all exponent bits set)
  {
    o.s.Exponent = 31;
    o.s.Mantissa = f.s.Mantissa ? 0x200 : 0;  // NaN->qNaN and Inf->Inf
  } else                                      // Normalized number
  {
    // Exponent unbias the single, then bias the halfp
    int newexp = f.s.Exponent - 127 + 15;
    if (newexp >= 31)  // Overflow, return signed infinity
      o.s.Exponent = 31;
    else if (newexp <= 0)  // Underflow
    {
      if ((14 - newexp) <= 24)  // Mantissa might be non-zero
      {
        unsigned int mant = f.s.Mantissa | 0x800000;  // Hidden 1 bit
        o.s.Mantissa = mant >> (14 - newexp);
        if ((mant >> (13 - newexp)) & 1)  // Check for rounding
          o.u++;  // Round, might overflow into exp bit, but this is OK
      }
    } else {
      o.s.Exponent = static_cast<unsigned int>(newexp);
      o.s.Mantissa = f.s.Mantissa >> 13;
      if (f.s.Mantissa & 0x1000)  // Check for rounding
        o.u++;                    // Round, might overflow to inf, this is OK
    }
  }

  o.s.Sign = f.s.Sign;
  return o;
}

static inline void swap2(unsigned short *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  unsigned short tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[1];
  dst[1] = src[0];
#endif
}

static inline void swap4(unsigned int *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  unsigned int tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
#endif
}

static inline void swap4(int *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  int tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
#endif
}

static inline void swap8(tinyvdb::tinyvdb_uint64 *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  tinyvdb::tinyvdb_uint64 tmp = (*val);
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[7];
  dst[1] = src[6];
  dst[2] = src[5];
  dst[3] = src[4];
  dst[4] = src[3];
  dst[5] = src[2];
  dst[6] = src[1];
  dst[7] = src[0];
#endif
}

static inline void swap8(tinyvdb::tinyvdb_int64 *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  tinyvdb::tinyvdb_int64 tmp = (*val);
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[7];
  dst[1] = src[6];
  dst[2] = src[5];
  dst[3] = src[4];
  dst[4] = src[3];
  dst[5] = src[2];
  dst[6] = src[1];
  dst[7] = src[0];
#endif
}

static Value ReadValue(std::istream &is, const ValueType type) {
  if (type == VALUE_TYPE_NULL) {
    return Value();
  } else if (type == VALUE_TYPE_BOOL) {
    char value;
    is.read(&value, 1);
    return Value(value);
  } else if (type == VALUE_TYPE_FLOAT) {
    float value;
    is.read(reinterpret_cast<char *>(&value), sizeof(float));
    swap4(reinterpret_cast<int *>(&value));
    return Value(value);
  } else if (type == VALUE_TYPE_INT) {
    int value;
    is.read(reinterpret_cast<char *>(&value), sizeof(int));
    swap4(&value);
    return Value(value);
  } else if (type == VALUE_TYPE_DOUBLE) {
    double value;
    is.read(reinterpret_cast<char *>(&value), sizeof(double));
    swap8(reinterpret_cast<tinyvdb_int64 *>(&value));
    return Value(value);
  }
  // ???
  return Value();
}

static inline std::string ReadString(std::istream &is) {
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  std::string buffer(size, ' ');
  if (size > 0) is.read(&buffer[0], size);
  return buffer;
}

static inline void WriteString(std::ostream &os, const std::string &name) {
  unsigned int size = static_cast<unsigned int>(name.size());
  os.write(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  os.write(&name[0], size);
}

static inline bool ReadMetaBool(std::istream &is) {
  char c = 0;
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == 1) {
    is.read(&c, 1);
  }
  return bool(c);
}

static inline float ReadMetaFloat(std::istream &is) {
  float f = 0.0f;
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == sizeof(float)) {
    is.read(reinterpret_cast<char *>(&f), sizeof(float));
    swap4(reinterpret_cast<unsigned int *>(&f));
  }
  return f;
}

static inline void ReadMetaVec3i(std::istream &is, int v[3]) {
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == 3 * sizeof(int)) {
    is.read(reinterpret_cast<char *>(v), 3 * sizeof(int));
    swap4(&v[0]);
    swap4(&v[1]);
    swap4(&v[2]);
  }
}

static inline void ReadVec3d(std::istream &is, double v[3]) {
  is.read(reinterpret_cast<char *>(v), 3 * sizeof(double));
  swap8(reinterpret_cast<tinyvdb_int64 *>(&v[0]));
  swap8(reinterpret_cast<tinyvdb_int64 *>(&v[1]));
  swap8(reinterpret_cast<tinyvdb_int64 *>(&v[2]));
}

static inline tinyvdb_int64 ReadMetaInt64(std::istream &is) {
  tinyvdb_int64 i64 = 0;
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == 8) {
    is.read(reinterpret_cast<char *>(&i64), 8);
    swap8(&i64);
  }
  return i64;
}

// https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
static inline bool EndsWidth(std::string const &value,
                             std::string const &ending) {
  if (ending.size() > value.size()) return false;
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

bool RootNode::ReadTopology(std::istream &is, const bool from_half,
                            const unsigned int file_version) {
  {
    int buffer_count;
    is.read(reinterpret_cast<char *>(&buffer_count), sizeof(int));
    if (buffer_count != 1) {
      // OPENVDB_LOG_WARN("multi-buffer trees are no longer supported");
    }
  }

  // Read background value;
  background_ = ReadValue(is, node_info_.value_type());

  std::cout << "background : " << background_ << std::endl;

  unsigned int num_tiles = 0;
  unsigned int num_children = 0;
  is.read(reinterpret_cast<char *>(&num_tiles), sizeof(unsigned int));
  swap4(&num_tiles);
  is.read(reinterpret_cast<char *>(&num_children), sizeof(unsigned int));
  swap4(&num_children);

  if ((num_tiles == 0) && (num_children == 0)) {
    return false;
  }

  std::cout << "num_tiles " << num_tiles << std::endl;
  std::cout << "num_children " << num_children << std::endl;

  // Read tiles.
  for (unsigned int n = 0; n < num_tiles; n++) {
    int vec[3];
    Value value;
    bool active;

    is.read(reinterpret_cast<char *>(vec), 3 * sizeof(int));
    value = ReadValue(is, node_info_.value_type());
    is.read(reinterpret_cast<char *>(&active), sizeof(bool));
    swap4(&vec[0]);
    swap4(&vec[1]);
    swap4(&vec[2]);

    std::cout << "[" << n << "] vec = (" << vec[0] << ", " << vec[1] << ", "
              << vec[2] << "), value = " << value << ", active = " << active
              << std::endl;
  }

  // Read child nodes.
  for (unsigned int n = 0; n < num_children; n++) {
    int vec[3];
    is.read(reinterpret_cast<char *>(vec), 3 * sizeof(int));

    swap4(&vec[0]);
    swap4(&vec[1]);
    swap4(&vec[2]);

    child_node_->ReadTopology(is, from_half, file_version);

#if 0  // TODO
    //mTable[Coord(vec)] = NodeStruct(*child);
#endif
  }

  (void)file_version;

  return true;
}

bool ReadCompressedValues(std::istream &is,
  const unsigned int compression_flags,
  const int file_version,
  size_t num_values,
  ValueType value_type,
  NodeMask value_mask,
  const bool from_half,
  std::vector<unsigned char> *values)
{
  const bool mask_compressed = compression_flags & COMPRESS_ACTIVE_MASK;

  char metadata = 0;
    if (file_version >= OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION) {
        // Read the flag that specifies what, if any, additional metadata
        // (selection mask and/or inactive value(s)) is saved.
        if ((!values) && !mask_compressed) {
            is.seekg(/*bytes=*/1, std::ios_base::cur);
        } else {
            is.read(reinterpret_cast<char*>(&metadata), /*bytes=*/1);
        }
    }


  return false;
}

bool InternalNode::ReadTopology(std::istream &is, const bool from_half,
                                const unsigned int file_version) {
  (void)from_half;
  (void)file_version;
  (void)child_node_;

  {
    int buffer_count;
    is.read(reinterpret_cast<char *>(&buffer_count), sizeof(int));
    if (buffer_count != 1) {
      // OPENVDB_LOG_WARN("multi-buffer trees are no longer supported");
    }
  }

  child_mask_.load(is);
  value_mask_.load(is);

  const bool old_version =
      file_version < OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION;

  Index NUM_VALUES = 1 << (3 * node_info_.log2dim());  // total voxel count represented by this node

  const Index num_values = (old_version ? Index(child_mask_.countOff()) : NUM_VALUES);

  {
      // Read in (and uncompress, if necessary) all of this node's values
      // into a contiguous array.
      //std::unique_ptr<ValueType[]> valuePtr(new ValueType[numValues]);
      //ValueType* values = valuePtr.get();
      std::vector<unsigned char> values;
      values.resize(GetValueTypeSize(node_info_.value_type()) * size_t(num_values));
      ReadCompressedValues(is, num_values, value_type, value_mask_, from_half, &values);

      // Copy values from the array into this node's table.
      if (old_version) {
          //Index n = 0;
          //for (ValueAllIter iter = this->beginValueAll(); iter; ++iter) {
          //    mNodes[iter.pos()].setValue(values[n++]);
          //}
          //assert(n == numValues);
      } else {
          //for (ValueAllIter iter = this->beginValueAll(); iter; ++iter) {
          //    mNodes[iter.pos()].setValue(values[iter.pos()]);
          //}
      }
  }

#if 0
  // Read in all child nodes and insert them into the table at their proper locations.
  for (ChildOnIter iter = this->beginChildOn(); iter; ++iter) {
      ChildNodeType* child = new ChildNodeType(PartialCreate(), iter.getCoord(), background);
      mNodes[iter.pos()].setChild(child);
      child->readTopology(is, fromHalf);
  }
#endif

  return true;
}

#if 0
template<typename ChildT>
bool TreeReader::Read(std::istream &is)
{
  
}
#endif

GridDescriptor::GridDescriptor()
    : save_float_as_half_(false), grid_pos_(0), block_pos_(0), end_pos_(0) {}

GridDescriptor::GridDescriptor(const std::string &name,
                               const std::string &grid_type,
                               bool save_float_as_half)
    : grid_name_(StripSuffix(name)),
      unique_name_(name),
      grid_type_(grid_type),
      save_float_as_half_(save_float_as_half),
      grid_pos_(0),
      block_pos_(0),
      end_pos_(0) {}

// GridDescriptor::GridDescriptor(const GridDescriptor &rhs) {
//}

GridDescriptor::~GridDescriptor() {}

std::string GridDescriptor::AddSuffix(const std::string &name, int n) {
  std::ostringstream ss;
  ss << name << SEP << n;
  return ss.str();
}

std::string GridDescriptor::StripSuffix(const std::string &name) {
  return name.substr(0, name.find(SEP));
}

bool GridDescriptor::Read(std::ifstream &is, const unsigned int file_version,
                          std::string *err) {
  unique_name_ = ReadString(is);
  grid_name_ = StripSuffix(unique_name_);

  grid_type_ = ReadString(is);

  if (EndsWidth(grid_type_, HALF_FLOAT_TYPENAME_SUFFIX)) {
    save_float_as_half_ = true;
    // strip suffix
    std::string tmp =
        grid_type_.substr(0, grid_type_.find(HALF_FLOAT_TYPENAME_SUFFIX));
    grid_type_ = tmp;
  }

  // FIXME(syoyo): Currently only `Tree_float_5_4_3` is supported.
  if (grid_type_.compare("Tree_float_5_4_3") != 0) {
    if (err) {
      (*err) = "Unsupported grid type: " + grid_type_;
    }
    return false;
  }

  std::cout << "grid_type = " << grid_type_ << std::endl;
  std::cout << "half = " << save_float_as_half_ << std::endl;

  if (file_version >= OPENVDB_FILE_VERSION_GRID_INSTANCING) {
    instance_parent_name_ = ReadString(is);
    std::cout << "instance_parent_name = " << instance_parent_name_
              << std::endl;
  }

  // Create the grid of the type if it has been registered.
  // if (!GridBase::isRegistered(mGridType)) {
  //    OPENVDB_THROW(LookupError, "Cannot read grid." <<
  //        " Grid type " << mGridType << " is not registered.");
  //}
  // else
  // GridBase::Ptr grid = GridBase::createGrid(mGridType);
  // if (grid) grid->setSaveFloatAsHalf(mSaveFloatAsHalf);

  // Read in the offsets.
  is.read(reinterpret_cast<char *>(&grid_pos_), sizeof(tinyvdb_int64));
  is.read(reinterpret_cast<char *>(&block_pos_), sizeof(tinyvdb_int64));
  is.read(reinterpret_cast<char *>(&end_pos_), sizeof(tinyvdb_int64));

  std::cout << "grid_pos = " << grid_pos_ << std::endl;
  std::cout << "block_pos = " << block_pos_ << std::endl;
  std::cout << "end_pos = " << end_pos_ << std::endl;

  return true;
}

static bool ReadMeta(std::ifstream &is) {
  // Read the number of metadata items.
  int count = 0;
  is.read(reinterpret_cast<char *>(&count), sizeof(int));
  swap4(&count);

  std::cout << "meta = " << count << std::endl;

  for (int i = 0; i < count; i++) {
    std::string name = ReadString(is);

    // read typename string
    std::string type_name = ReadString(is);

    std::cout << "meta[" << i << "] name = " << name
              << ", type_name = " << type_name << std::endl;

    if (type_name.compare("string") == 0) {
      std::string value = ReadString(is);

      std::cout << "  value = " << value << std::endl;

    } else if (type_name.compare("vec3i") == 0) {
      int v[3];
      ReadMetaVec3i(is, v);

      std::cout << "  value = " << v[0] << ", " << v[1] << ", " << v[2]
                << std::endl;

    } else if (type_name.compare("bool") == 0) {
      bool b = ReadMetaBool(is);

      std::cout << "  value = " << b << std::endl;

    } else if (type_name.compare("float") == 0) {
      float f = ReadMetaFloat(is);

      std::cout << "  value = " << f << std::endl;

    } else if (type_name.compare("int64") == 0) {
      tinyvdb_int64 i64 = ReadMetaInt64(is);

      std::cout << "  value = " << i64 << std::endl;

    } else {
      // Unknown metadata
      int num_bytes;
      is.read(reinterpret_cast<char *>(&num_bytes), sizeof(int));
      swap4(&num_bytes);

      std::cout << "  unknown value. size = " << num_bytes << std::endl;

      std::vector<char> data;
      data.resize(size_t(num_bytes));
      is.read(data.data(), num_bytes);
    }
  }

  return true;
}

static void ReadGridDescriptors(std::ifstream &is,
                                const unsigned int file_version,
                                std::map<std::string, GridDescriptor> *gd_map) {
  // Read the number of metadata items.
  int count = 0;
  is.read(reinterpret_cast<char *>(&count), sizeof(int));

  std::cout << "grid descriptors = " << count << std::endl;

  for (int i = 0; i < count; ++i) {
    // Read the grid descriptor.
    GridDescriptor gd;
    std::string err;
    bool ret = gd.Read(is, file_version, &err);
    assert(ret);

    (*gd_map)[gd.GridName()] = gd;
#if 0
      // Add the descriptor to the dictionary.
      gridDescriptors().insert(std::make_pair(gd.gridName(), gd));

      // Skip forward to the next descriptor.
      gd.seekToEnd(is);
#endif
  }
}

static void ReadTransform(std::ifstream &is) {
  // Read the type name.
  std::string type = ReadString(is);

  std::cout << "transform type = " << type << std::endl;

  double scale_values[3];
  double voxel_size[3];
  double scale_values_inverse[3];
  double inv_scale_squared[3];
  double inv_twice_scale[3];

  if (type.compare("UniformScaleMap") == 0) {
    scale_values[0] = scale_values[1] = scale_values[2] = 0.0;
    voxel_size[0] = voxel_size[1] = voxel_size[2] = 0.0;
    scale_values_inverse[0] = scale_values_inverse[1] =
        scale_values_inverse[2] = 0.0;
    inv_scale_squared[0] = inv_scale_squared[1] = inv_scale_squared[2] = 0.0;
    inv_twice_scale[0] = inv_twice_scale[1] = inv_twice_scale[2] = 0.0;

    ReadVec3d(is, scale_values);
    ReadVec3d(is, voxel_size);
    ReadVec3d(is, scale_values_inverse);
    ReadVec3d(is, inv_scale_squared);
    ReadVec3d(is, inv_twice_scale);

    std::cout << "scale_values " << scale_values[0] << ", " << scale_values[1]
              << ", " << scale_values[2] << std::endl;
    std::cout << "voxel_size " << voxel_size[0] << ", " << voxel_size[1] << ", "
              << voxel_size[2] << std::endl;
    std::cout << "scale_value_sinverse " << scale_values_inverse[0] << ", "
              << scale_values_inverse[1] << ", " << scale_values_inverse[2]
              << std::endl;
    std::cout << "inv_scale_squared " << inv_scale_squared[0] << ", "
              << inv_scale_squared[1] << ", " << inv_scale_squared[2]
              << std::endl;
    std::cout << "inv_twice_scale " << inv_twice_scale[0] << ", "
              << inv_twice_scale[1] << ", " << inv_twice_scale[2] << std::endl;
  } else {
    assert(0);
    // TODO(syoyo): Implement
  }

  // TODO(syoyo) read transform
}

static void ReadGrid(std::ifstream &is, const unsigned int file_version,
                     const GridDescriptor &gd) {
  if (file_version >= OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION) {
    unsigned int c = COMPRESS_NONE;
    is.read(reinterpret_cast<char *>(&c), sizeof(unsigned int));
    std::cout << "compression: " << c << std::endl;
    // io::setDataCompression(is, c);
  }

  ReadMeta(is);

  // read transform
  ReadTransform(is);

  // read topology
  if (!gd.IsInstance()) {
    // TODO(syoyo): Construct node hierarchy based on header description.
    NodeInfo leaf(NODE_TYPE_LEAF, VALUE_TYPE_FLOAT, 3);
    NodeInfo internal2(NODE_TYPE_INTERNAL, VALUE_TYPE_FLOAT, 4);
    NodeInfo internal1(NODE_TYPE_INTERNAL, VALUE_TYPE_FLOAT, 5);
    NodeInfo root(NODE_TYPE_ROOT, VALUE_TYPE_FLOAT, 0);

    LeafNode leaf_node(leaf);
    InternalNode internal2_node(internal2, &leaf_node);
    InternalNode internal1_node(internal1, &internal2_node);
    RootNode root_node(root, &internal1_node);

    root_node.ReadTopology(is, gd.SaveFloatAsHalf(), file_version);
  }

  is.seekg(gd.GridPos());
}

bool ParseVDBHeader(const std::string &filename, std::string *err) {
  std::ifstream ifs(filename.c_str(), std::ifstream::binary);
  if (!ifs) {
    if (err) {
      (*err) = "File not found or cannot open file : " + filename;
    }
    return false;
  }

  int64_t magic;

  // [0:7] magic number
  if (!ifs.read(reinterpret_cast<char *>(&magic), 8)) {
    return EXIT_FAILURE;
  }

  if (magic == kOPENVDB_MAGIC) {
    std::cout << "bingo!" << std::endl;
  }

  // [8:11] version
  unsigned int file_version;
  if (!ifs.read(reinterpret_cast<char *>(&file_version),
                sizeof(unsigned int))) {
    return EXIT_FAILURE;
  }

  std::cout << "File version: " << file_version << std::endl;

  // Read the library version numbers (not stored prior to file format version
  // 211).
  if (file_version >= 211) {
    unsigned int version;
    ifs.read(reinterpret_cast<char *>(&version), sizeof(unsigned int));
    std::cout << "major version : " << version << std::endl;
    ifs.read(reinterpret_cast<char *>(&version), sizeof(unsigned int));
    std::cout << "minor version : " << version << std::endl;
  }

  // Read the flag indicating whether the stream supports partial reading.
  // (Versions prior to 212 have no flag because they always supported partial
  // reading.)
  char has_grid_offsets = 0;
  if (file_version >= 212) {
    ifs.read(&has_grid_offsets, sizeof(char));
    std::cout << "InputHasGridOffsets = "
              << (has_grid_offsets ? " yes " : " no ") << std::endl;
  }

  // 5) Read the flag that indicates whether data is compressed.
  //    (From version 222 on, compression information is stored per grid.)
  // mCompression = DEFAULT_COMPRESSION_FLAGS;
  // if (file_version < OPENVDB_FILE_VERSION_BLOSC_COMPRESSION) {
  //    // Prior to the introduction of Blosc, ZLIB was the default compression
  //    scheme. mCompression = (COMPRESS_ZIP | COMPRESS_ACTIVE_MASK);
  //}
  char isCompressed = 0;
  if (file_version >= OPENVDB_FILE_VERSION_SELECTIVE_COMPRESSION &&
      file_version < OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION) {
    ifs.read(&isCompressed, sizeof(char));
    std::cout << "Compression : " << (isCompressed != 0 ? "zip" : "none")
              << std::endl;
  }

  // 6) Read the 16-byte(128-bit) uuid.
  if (file_version >= OPENVDB_FILE_VERSION_BOOST_UUID) {
    // ASCII UUID = 32 chars + 4 '-''s = 36 bytes.
    char uuid[36];
    ifs.read(uuid, 36);
    std::string uuid_string = std::string(uuid, 36);
    // TODO(syoyo): Store UUID somewhere.
    std::cout << "uuid: " << uuid_string << std::endl;
  } else {
    char uuid[16];
    ifs.read(uuid, 16);
    // TODO(syoyo): Store UUID somewhere.
    std::cout << "uuid: " << uuid << std::endl;
  }

  {
    bool ret = ReadMeta(ifs);
    std::cout << "meta: " << ret << std::endl;
  }

  std::map<std::string, GridDescriptor> gd_map;

  if (has_grid_offsets) {
    ReadGridDescriptors(ifs, file_version, &gd_map);
  } else {
  }

  // fixme
  std::map<std::string, GridDescriptor>::iterator it(gd_map.begin());
  // std::map<std::string, GridDescriptor>::iterator itEnd(gd_map.end());

  // for (; it != itEnd; it++) {
  ReadGrid(ifs, file_version, it->second);
  //}

  return true;
}

bool ParseVDBHeader(const unsigned char *data, const size_t len,
                    std::string *err) {
  (void)data;
  (void)len;
  (void)err;

  return false;
}

static bool WriteVDBHeader(std::ostream &os) {
  // [0:7] magic number
  tinyvdb_int64 magic = kOPENVDB_MAGIC;
  os.write(reinterpret_cast<char *>(&magic), 8);

  // [8:11] version
  unsigned int file_version = OPENVDB_FILE_VERSION_NEW_TRANSFORM;
  os.write(reinterpret_cast<char *>(&file_version), sizeof(unsigned int));

#if 0

  std::cout << "File version: " << file_version << std::endl;

  // Read the library version numbers (not stored prior to file format version
  // 211).
  if (file_version >= 211) {
    unsigned int version;
    ifs.read(reinterpret_cast<char *>(&version), sizeof(unsigned int));
    std::cout << "major version : " << version << std::endl;
    ifs.read(reinterpret_cast<char *>(&version), sizeof(unsigned int));
    std::cout << "minor version : " << version << std::endl;
  }

  // Read the flag indicating whether the stream supports partial reading.
  // (Versions prior to 212 have no flag because they always supported partial
  // reading.)
  char has_grid_offsets = 0;
  if (file_version >= 212) {
    ifs.read(&has_grid_offsets, sizeof(char));
    std::cout << "InputHasGridOffsets = " << (has_grid_offsets ? " yes " : " no ")
              << std::endl;
  }

  // 5) Read the flag that indicates whether data is compressed.
  //    (From version 222 on, compression information is stored per grid.)
  // mCompression = DEFAULT_COMPRESSION_FLAGS;
  // if (file_version < OPENVDB_FILE_VERSION_BLOSC_COMPRESSION) {
  //    // Prior to the introduction of Blosc, ZLIB was the default compression
  //    scheme. mCompression = (COMPRESS_ZIP | COMPRESS_ACTIVE_MASK);
  //}
  char isCompressed = 0;
  if (file_version >= OPENVDB_FILE_VERSION_SELECTIVE_COMPRESSION &&
      file_version < OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION) {
    ifs.read(&isCompressed, sizeof(char));
    std::cout << "Compression : " << (isCompressed != 0 ? "zip" : "none")
              << std::endl;
  }

  // 6) Read the 16-byte(128-bit) uuid.
  if (file_version >= OPENVDB_FILE_VERSION_BOOST_UUID) {
    // ASCII UUID = 32 chars + 4 '-''s = 36 bytes.
    char uuid[36];
    ifs.read(uuid, 36);
    // TODO(syoyo): Store UUID somewhere.
    std::cout << "uuid: " << uuid << std::endl;
  } else {
    char uuid[16];
    ifs.read(uuid, 16);
    // TODO(syoyo): Store UUID somewhere.
    std::cout << "uuid: " << uuid << std::endl;
  }

  {
    bool ret = ReadMeta(ifs);
    std::cout << "meta: " << ret << std::endl;
  }

  if (has_grid_offsets) {
    ReadGridDescriptors(ifs);
  } else {
  }
#endif

  return true;
}

bool SaveVDB(const std::string &filename, std::string *err) {
  std::ofstream os(filename.c_str(), std::ifstream::binary);

  if (!os) {
    if (err) {
      (*err) = "Failed to open a file to write: " + filename;
    }
    return false;
  }

  WriteVDBHeader(os);
  // if filemane

  return true;
}

}  // namespace tinyvdb

#endif

#endif  // TINY_VDB_H_

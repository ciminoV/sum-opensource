/**
 * [2024] SubSeaPulse SRL 
 * All Rights Reserved.
 */

// This is free software: you can redistribute it and/or modify it        *
// under the terms of the GNU General Public License version 3 as         *
// published by the Free Software Foundation.                             *
//                                                                        *
// This program is distributed in the hope that it will be useful, but    *
// WITHOUT ANY WARRANTY; without even the implied warranty of FITNESS     *
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for       *
// more details.                                                          *
//                                                                        *
// You should have received a copy of the GNU General Public License      *
// along with this program. If not, see <http://www.gnu.org/licenses/>.   *

#include <chunk.h>
#include <string.h>
const size_t Chunk::DEFAULT_SIZE = 128;
const size_t Chunk::MAX_HEADER_SIZE = sizeof(MacHdr); 

Chunk::Chunk(std::size_t max_bytes)
:
chunk_size(0),
data_chunk(max_bytes),// not use {} here due to vector list initializer
chunk_hdr(Chunk::MAX_HEADER_SIZE),
hdr_rev_offset(Chunk::MAX_HEADER_SIZE),
modulated(false),
samples(nullptr)
{

}

Chunk::~Chunk()
{

}

std::size_t Chunk::getMaxSize() const
{
  return data_chunk.size();
}

std::size_t Chunk::getSize() const
{
  return chunk_size;
}

bool Chunk::isFull() const
{
  return chunk_size >= data_chunk.size();
}

char* Chunk::data()
{
  return data_chunk.data();
}

void Chunk::setSize(std::size_t sz)
{
  chunk_size = sz > data_chunk.size() ? data_chunk.size() : sz;
}

void Chunk::setModulatedSamples(std::shared_ptr<Samples> samp)
{
  if(samp != nullptr) {
    samples = samp;
    modulated = true;
  }  
}

void Chunk::resetModulatedSamples()
{
  samples = nullptr;
  modulated = false;  
}

bool Chunk::isModulated() const
{
  return modulated;
}

std::shared_ptr<Samples> Chunk::getModulatedSamples() const
{
  return samples;
}

bool Chunk::addHeader(const void* hdr, size_t size)
{
  if((hdr_rev_offset - size) < 0 || size <= 0) {
    return false;
  }
  memcpy(&chunk_hdr[hdr_rev_offset - size],hdr,size);
  hdr_rev_offset-=size;
  return true;
}

bool Chunk::deserializeHeader(void* hdr, size_t size)
{
  if((hdr_rev_offset + size) > chunk_hdr.size() || size <= 0) {
    return false;
  }
  memcpy(hdr,&chunk_hdr[hdr_rev_offset],size);
  hdr_rev_offset+=size;
  return true;
}

const char* Chunk::getHeader() const
{
  return &chunk_hdr[hdr_rev_offset];
}

size_t Chunk::getHeaderSize() const
{
  return chunk_hdr.size() - hdr_rev_offset;
}

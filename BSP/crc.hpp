#ifndef _PROBABILITY_CRC_H_
#define _PROBABILITY_CRC_H_

#include <cstdint>

namespace probability
{
	class CRC
	{
	public:
		CRC(uint32_t polynomial, uint8_t width, uint32_t initial_xor, uint32_t final_xor, bool reflect_data, bool reflect_remainder) :
			polynomial(polynomial),
			width(width),
			initial_xor(initial_xor),
			final_xor(final_xor),
			_reflect_data(reflect_data),
			_reflect_remainder(reflect_remainder)
		{
			;
		}

		virtual ~CRC() { ; }

		const int32_t compute(const void* buffer, const uint64_t size, uint32_t* const crc) const
		{
			if (crc == nullptr) { return 1; }

			int32_t status = 0;

			status = compute_partial_start(crc);
			if (0 != status) { return status; }

			status = compute_partial(buffer, size, crc);
			if (0 != status) { return status; }

			status = compute_partial_stop(crc);
			if (0 != status) { *crc = 0; }

			return status;
		}

		const int32_t compute_partial(const void* buffer, const uint64_t size, uint32_t* const crc) const
		{
			if (crc == nullptr) { return 1; }

			const uint8_t* data = static_cast<const uint8_t*>(buffer);
			uint32_t _crc = *crc;

			if (width < 8)
			{
				uint8_t data_byte;
				for (uint64_t byte = 0; byte < size; byte++)
				{
					data_byte = reflect_data(data[byte]);
					for (uint8_t bit = 8; bit > 0; bit--)
					{
						_crc = _crc << 1;
						if ((data_byte ^ _crc) & get_top_bit())
						{
							_crc ^= polynomial;
						}
						data_byte = data_byte << 1;
					}
				}
			}
			else
			{
				for (uint64_t byte = 0; byte < size; byte++)
				{
					_crc ^= (reflect_data(data[byte]) << (width - 8));

					for (uint8_t bit = 8; bit > 0; bit--)
					{
						if (_crc & get_top_bit())
						{
							_crc = (_crc << 1) ^ polynomial;
						}
						else
						{
							_crc = (_crc << 1);
						}
					}
				}
			}

			*crc = _crc & get_crc_mask();

			return 0;
		}

		const int32_t compute_partial_start(uint32_t* const crc) const
		{
			if (crc == nullptr) { return 1; }

			*crc = initial_xor;
			return 0;
		}

		const int32_t compute_partial_stop(uint32_t* const crc) const
		{
			if (crc == nullptr) { return 1; }

			uint32_t _crc = *crc;
			if (width < 8)
			{
				_crc = (uint32_t)(_crc << (8 - width));
			}

			*crc = (reflect_remainder(_crc) ^ final_xor) & get_crc_mask();

			return 0;
		}

		const uint32_t get_polynomial() const
		{
			return polynomial;
		}

		const uint8_t get_width() const
		{
			return width;
		}

	private:
		const uint32_t polynomial;
		const uint8_t width;
		const uint32_t initial_xor;
		const uint32_t final_xor;
		const bool _reflect_data;
		const bool _reflect_remainder;

		const uint32_t get_top_bit() const
		{
			return ((width < 8) ? (1u << 7) : (uint32_t)(1ul << (width - 1)));
		}

		const uint32_t get_crc_mask() const
		{
			return ((width < 8) ? ((1u << 8) - 1) : (uint32_t)((uint64_t)(1ull << width) - 1));
		}

		const uint32_t reflect_remainder(const uint32_t data) const
		{
			if (_reflect_remainder)
			{
				uint32_t reflection = 0x0;
				uint32_t _data = data;
				const uint8_t nbits = (width < 8) ? 8 : width;

				for (uint8_t bit = 0; bit < nbits; bit++)
				{
					if (_data & 0x01)
					{
						reflection |= (1 << ((nbits - 1) - bit));
					}
					_data = (_data >> 1);
				}

				return reflection;
			}
			else
			{
				return data;
			}
		}

		const uint32_t reflect_data(const uint32_t data) const
		{
			if (_reflect_data)
			{
				uint32_t reflection = 0x0;
				uint32_t _data = data;

				for (uint8_t bit = 0; bit < 8; bit++)
				{
					if (_data & 0x01)
					{
						reflection |= (1 << (7 - bit));
					}
					_data = (_data >> 1);
				}

				return reflection;
			}
			else
			{
				return data;
			}
		}
	};
} // namespace probability

#endif // _PROBABILITY_CRC_H_
#include <cctype>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <ostream>

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <string>

#include <vector>

// Apache-2.0 license
// GITHUB：https://github.com/eyjian/r3c

namespace StringUtils
{
    std::string ltrim(const std::string &str)
    {
        int i = 0;
        while (i < int(str.size()) && isspace(str[i]))
            i++;
        return str.substr(i);
    }

    std::string rtrim(const std::string &str)
    {
        int i = int(str.size()) - 1;
        while (i >= 0 && isspace(str[i]))
            i--;
        return str.substr(0, i + 1);
    }

    std::string trim(const std::string &str)
    {
        return rtrim(ltrim(str));
    }

    std::string escape(const std::string &str, char c)
    {
        std::string escaped_str;
        for (int i = 0; i < int(str.size()); i++)
        {
            if (str[i] == c)
                escaped_str += "\\";
            escaped_str += std::string(1, str[i]);
        }
        return escaped_str;
    }

    std::string unescape(const std::string &str, char c)
    {
        std::string unescaped_str;
        for (int i = 0; i < int(str.size()); i++)
        {
            if (i < int(str.size()) - 1 && str[i] == '\\' && str[i + 1] == c)
                continue;
            unescaped_str += std::string(1, str[i]);
        }
        return unescaped_str;
    }

    bool is_space(char c)
    {
        return isspace(c);
    }

    bool is_alpha(char c)
    {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
    }

    bool is_numeric(char c)
    {
        return (c >= '0' && c <= '9');
    }

    bool is_alphanumeric(char c)
    {
        return is_alpha(c) || is_numeric(c);
    }

    bool starts_with(const std::string &str, const std::string &prefix)
    {
        if (str.size() < prefix.size())
            return false;
        for (int i = 0; i < int(prefix.size()); i++)
            if (str[i] != prefix[i])
                return false;
        return true;
    }

    bool ends_with(const std::string &str, const std::string &suffix)
    {
        if (str.size() < suffix.size())
            return false;
        for (int i = 0; i < int(suffix.size()); i++)
            if (str[int(str.size()) - i - 1] != suffix[int(suffix.size()) - i - 1])
                return false;
        return true;
    }

    std::string remove_spaces(const std::string &str)
    {
        std::string str_without_spaces;
        for (int i = 0; i < int(str.size()); i++)
        {
            if (is_space(str[i]))
                continue;
            str_without_spaces += std::string(1, str[i]);
        }
        return str_without_spaces;
    }

    /* Copied from redis source code (crc16.c)
     *
     * CRC16 implementation according to CCITT standards.
     *
     * Note by @antirez: this is actually the XMODEM CRC 16 algorithm, using the
     * following parameters:
     *
     * Name                       : "XMODEM", also known as "ZMODEM", "CRC-16/ACORN"
     * Width                      : 16 bit
     * Poly                       : 1021 (That is actually x^16 + x^12 + x^5 + 1)
     * Initialization             : 0000
     * Reflect Input byte         : False
     * Reflect Output CRC         : False
     * Xor constant to output CRC : 0000
     * Output for "123456789"     : 31C3
     */

    static const uint16_t crc16tab[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

    /* Copied from redis source code (crc16.c)
     */
    uint16_t crc16(const char *buf, int len)
    {
        int counter;
        uint16_t crc = 0;
        for (counter = 0; counter < len; counter++)
            crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ *buf++) & 0x00FF];
        return crc;
    }

    /* Copied from redis source code (crc64.c)
     */
    static const uint64_t crc64_tab[256] = {
        UINT64_C(0x0000000000000000),
        UINT64_C(0x7ad870c830358979),
        UINT64_C(0xf5b0e190606b12f2),
        UINT64_C(0x8f689158505e9b8b),
        UINT64_C(0xc038e5739841b68f),
        UINT64_C(0xbae095bba8743ff6),
        UINT64_C(0x358804e3f82aa47d),
        UINT64_C(0x4f50742bc81f2d04),
        UINT64_C(0xab28ecb46814fe75),
        UINT64_C(0xd1f09c7c5821770c),
        UINT64_C(0x5e980d24087fec87),
        UINT64_C(0x24407dec384a65fe),
        UINT64_C(0x6b1009c7f05548fa),
        UINT64_C(0x11c8790fc060c183),
        UINT64_C(0x9ea0e857903e5a08),
        UINT64_C(0xe478989fa00bd371),
        UINT64_C(0x7d08ff3b88be6f81),
        UINT64_C(0x07d08ff3b88be6f8),
        UINT64_C(0x88b81eabe8d57d73),
        UINT64_C(0xf2606e63d8e0f40a),
        UINT64_C(0xbd301a4810ffd90e),
        UINT64_C(0xc7e86a8020ca5077),
        UINT64_C(0x4880fbd87094cbfc),
        UINT64_C(0x32588b1040a14285),
        UINT64_C(0xd620138fe0aa91f4),
        UINT64_C(0xacf86347d09f188d),
        UINT64_C(0x2390f21f80c18306),
        UINT64_C(0x594882d7b0f40a7f),
        UINT64_C(0x1618f6fc78eb277b),
        UINT64_C(0x6cc0863448deae02),
        UINT64_C(0xe3a8176c18803589),
        UINT64_C(0x997067a428b5bcf0),
        UINT64_C(0xfa11fe77117cdf02),
        UINT64_C(0x80c98ebf2149567b),
        UINT64_C(0x0fa11fe77117cdf0),
        UINT64_C(0x75796f2f41224489),
        UINT64_C(0x3a291b04893d698d),
        UINT64_C(0x40f16bccb908e0f4),
        UINT64_C(0xcf99fa94e9567b7f),
        UINT64_C(0xb5418a5cd963f206),
        UINT64_C(0x513912c379682177),
        UINT64_C(0x2be1620b495da80e),
        UINT64_C(0xa489f35319033385),
        UINT64_C(0xde51839b2936bafc),
        UINT64_C(0x9101f7b0e12997f8),
        UINT64_C(0xebd98778d11c1e81),
        UINT64_C(0x64b116208142850a),
        UINT64_C(0x1e6966e8b1770c73),
        UINT64_C(0x8719014c99c2b083),
        UINT64_C(0xfdc17184a9f739fa),
        UINT64_C(0x72a9e0dcf9a9a271),
        UINT64_C(0x08719014c99c2b08),
        UINT64_C(0x4721e43f0183060c),
        UINT64_C(0x3df994f731b68f75),
        UINT64_C(0xb29105af61e814fe),
        UINT64_C(0xc849756751dd9d87),
        UINT64_C(0x2c31edf8f1d64ef6),
        UINT64_C(0x56e99d30c1e3c78f),
        UINT64_C(0xd9810c6891bd5c04),
        UINT64_C(0xa3597ca0a188d57d),
        UINT64_C(0xec09088b6997f879),
        UINT64_C(0x96d1784359a27100),
        UINT64_C(0x19b9e91b09fcea8b),
        UINT64_C(0x636199d339c963f2),
        UINT64_C(0xdf7adabd7a6e2d6f),
        UINT64_C(0xa5a2aa754a5ba416),
        UINT64_C(0x2aca3b2d1a053f9d),
        UINT64_C(0x50124be52a30b6e4),
        UINT64_C(0x1f423fcee22f9be0),
        UINT64_C(0x659a4f06d21a1299),
        UINT64_C(0xeaf2de5e82448912),
        UINT64_C(0x902aae96b271006b),
        UINT64_C(0x74523609127ad31a),
        UINT64_C(0x0e8a46c1224f5a63),
        UINT64_C(0x81e2d7997211c1e8),
        UINT64_C(0xfb3aa75142244891),
        UINT64_C(0xb46ad37a8a3b6595),
        UINT64_C(0xceb2a3b2ba0eecec),
        UINT64_C(0x41da32eaea507767),
        UINT64_C(0x3b024222da65fe1e),
        UINT64_C(0xa2722586f2d042ee),
        UINT64_C(0xd8aa554ec2e5cb97),
        UINT64_C(0x57c2c41692bb501c),
        UINT64_C(0x2d1ab4dea28ed965),
        UINT64_C(0x624ac0f56a91f461),
        UINT64_C(0x1892b03d5aa47d18),
        UINT64_C(0x97fa21650afae693),
        UINT64_C(0xed2251ad3acf6fea),
        UINT64_C(0x095ac9329ac4bc9b),
        UINT64_C(0x7382b9faaaf135e2),
        UINT64_C(0xfcea28a2faafae69),
        UINT64_C(0x8632586aca9a2710),
        UINT64_C(0xc9622c4102850a14),
        UINT64_C(0xb3ba5c8932b0836d),
        UINT64_C(0x3cd2cdd162ee18e6),
        UINT64_C(0x460abd1952db919f),
        UINT64_C(0x256b24ca6b12f26d),
        UINT64_C(0x5fb354025b277b14),
        UINT64_C(0xd0dbc55a0b79e09f),
        UINT64_C(0xaa03b5923b4c69e6),
        UINT64_C(0xe553c1b9f35344e2),
        UINT64_C(0x9f8bb171c366cd9b),
        UINT64_C(0x10e3202993385610),
        UINT64_C(0x6a3b50e1a30ddf69),
        UINT64_C(0x8e43c87e03060c18),
        UINT64_C(0xf49bb8b633338561),
        UINT64_C(0x7bf329ee636d1eea),
        UINT64_C(0x012b592653589793),
        UINT64_C(0x4e7b2d0d9b47ba97),
        UINT64_C(0x34a35dc5ab7233ee),
        UINT64_C(0xbbcbcc9dfb2ca865),
        UINT64_C(0xc113bc55cb19211c),
        UINT64_C(0x5863dbf1e3ac9dec),
        UINT64_C(0x22bbab39d3991495),
        UINT64_C(0xadd33a6183c78f1e),
        UINT64_C(0xd70b4aa9b3f20667),
        UINT64_C(0x985b3e827bed2b63),
        UINT64_C(0xe2834e4a4bd8a21a),
        UINT64_C(0x6debdf121b863991),
        UINT64_C(0x1733afda2bb3b0e8),
        UINT64_C(0xf34b37458bb86399),
        UINT64_C(0x8993478dbb8deae0),
        UINT64_C(0x06fbd6d5ebd3716b),
        UINT64_C(0x7c23a61ddbe6f812),
        UINT64_C(0x3373d23613f9d516),
        UINT64_C(0x49aba2fe23cc5c6f),
        UINT64_C(0xc6c333a67392c7e4),
        UINT64_C(0xbc1b436e43a74e9d),
        UINT64_C(0x95ac9329ac4bc9b5),
        UINT64_C(0xef74e3e19c7e40cc),
        UINT64_C(0x601c72b9cc20db47),
        UINT64_C(0x1ac40271fc15523e),
        UINT64_C(0x5594765a340a7f3a),
        UINT64_C(0x2f4c0692043ff643),
        UINT64_C(0xa02497ca54616dc8),
        UINT64_C(0xdafce7026454e4b1),
        UINT64_C(0x3e847f9dc45f37c0),
        UINT64_C(0x445c0f55f46abeb9),
        UINT64_C(0xcb349e0da4342532),
        UINT64_C(0xb1eceec59401ac4b),
        UINT64_C(0xfebc9aee5c1e814f),
        UINT64_C(0x8464ea266c2b0836),
        UINT64_C(0x0b0c7b7e3c7593bd),
        UINT64_C(0x71d40bb60c401ac4),
        UINT64_C(0xe8a46c1224f5a634),
        UINT64_C(0x927c1cda14c02f4d),
        UINT64_C(0x1d148d82449eb4c6),
        UINT64_C(0x67ccfd4a74ab3dbf),
        UINT64_C(0x289c8961bcb410bb),
        UINT64_C(0x5244f9a98c8199c2),
        UINT64_C(0xdd2c68f1dcdf0249),
        UINT64_C(0xa7f41839ecea8b30),
        UINT64_C(0x438c80a64ce15841),
        UINT64_C(0x3954f06e7cd4d138),
        UINT64_C(0xb63c61362c8a4ab3),
        UINT64_C(0xcce411fe1cbfc3ca),
        UINT64_C(0x83b465d5d4a0eece),
        UINT64_C(0xf96c151de49567b7),
        UINT64_C(0x76048445b4cbfc3c),
        UINT64_C(0x0cdcf48d84fe7545),
        UINT64_C(0x6fbd6d5ebd3716b7),
        UINT64_C(0x15651d968d029fce),
        UINT64_C(0x9a0d8ccedd5c0445),
        UINT64_C(0xe0d5fc06ed698d3c),
        UINT64_C(0xaf85882d2576a038),
        UINT64_C(0xd55df8e515432941),
        UINT64_C(0x5a3569bd451db2ca),
        UINT64_C(0x20ed197575283bb3),
        UINT64_C(0xc49581ead523e8c2),
        UINT64_C(0xbe4df122e51661bb),
        UINT64_C(0x3125607ab548fa30),
        UINT64_C(0x4bfd10b2857d7349),
        UINT64_C(0x04ad64994d625e4d),
        UINT64_C(0x7e7514517d57d734),
        UINT64_C(0xf11d85092d094cbf),
        UINT64_C(0x8bc5f5c11d3cc5c6),
        UINT64_C(0x12b5926535897936),
        UINT64_C(0x686de2ad05bcf04f),
        UINT64_C(0xe70573f555e26bc4),
        UINT64_C(0x9ddd033d65d7e2bd),
        UINT64_C(0xd28d7716adc8cfb9),
        UINT64_C(0xa85507de9dfd46c0),
        UINT64_C(0x273d9686cda3dd4b),
        UINT64_C(0x5de5e64efd965432),
        UINT64_C(0xb99d7ed15d9d8743),
        UINT64_C(0xc3450e196da80e3a),
        UINT64_C(0x4c2d9f413df695b1),
        UINT64_C(0x36f5ef890dc31cc8),
        UINT64_C(0x79a59ba2c5dc31cc),
        UINT64_C(0x037deb6af5e9b8b5),
        UINT64_C(0x8c157a32a5b7233e),
        UINT64_C(0xf6cd0afa9582aa47),
        UINT64_C(0x4ad64994d625e4da),
        UINT64_C(0x300e395ce6106da3),
        UINT64_C(0xbf66a804b64ef628),
        UINT64_C(0xc5bed8cc867b7f51),
        UINT64_C(0x8aeeace74e645255),
        UINT64_C(0xf036dc2f7e51db2c),
        UINT64_C(0x7f5e4d772e0f40a7),
        UINT64_C(0x05863dbf1e3ac9de),
        UINT64_C(0xe1fea520be311aaf),
        UINT64_C(0x9b26d5e88e0493d6),
        UINT64_C(0x144e44b0de5a085d),
        UINT64_C(0x6e963478ee6f8124),
        UINT64_C(0x21c640532670ac20),
        UINT64_C(0x5b1e309b16452559),
        UINT64_C(0xd476a1c3461bbed2),
        UINT64_C(0xaeaed10b762e37ab),
        UINT64_C(0x37deb6af5e9b8b5b),
        UINT64_C(0x4d06c6676eae0222),
        UINT64_C(0xc26e573f3ef099a9),
        UINT64_C(0xb8b627f70ec510d0),
        UINT64_C(0xf7e653dcc6da3dd4),
        UINT64_C(0x8d3e2314f6efb4ad),
        UINT64_C(0x0256b24ca6b12f26),
        UINT64_C(0x788ec2849684a65f),
        UINT64_C(0x9cf65a1b368f752e),
        UINT64_C(0xe62e2ad306bafc57),
        UINT64_C(0x6946bb8b56e467dc),
        UINT64_C(0x139ecb4366d1eea5),
        UINT64_C(0x5ccebf68aecec3a1),
        UINT64_C(0x2616cfa09efb4ad8),
        UINT64_C(0xa97e5ef8cea5d153),
        UINT64_C(0xd3a62e30fe90582a),
        UINT64_C(0xb0c7b7e3c7593bd8),
        UINT64_C(0xca1fc72bf76cb2a1),
        UINT64_C(0x45775673a732292a),
        UINT64_C(0x3faf26bb9707a053),
        UINT64_C(0x70ff52905f188d57),
        UINT64_C(0x0a2722586f2d042e),
        UINT64_C(0x854fb3003f739fa5),
        UINT64_C(0xff97c3c80f4616dc),
        UINT64_C(0x1bef5b57af4dc5ad),
        UINT64_C(0x61372b9f9f784cd4),
        UINT64_C(0xee5fbac7cf26d75f),
        UINT64_C(0x9487ca0fff135e26),
        UINT64_C(0xdbd7be24370c7322),
        UINT64_C(0xa10fceec0739fa5b),
        UINT64_C(0x2e675fb4576761d0),
        UINT64_C(0x54bf2f7c6752e8a9),
        UINT64_C(0xcdcf48d84fe75459),
        UINT64_C(0xb71738107fd2dd20),
        UINT64_C(0x387fa9482f8c46ab),
        UINT64_C(0x42a7d9801fb9cfd2),
        UINT64_C(0x0df7adabd7a6e2d6),
        UINT64_C(0x772fdd63e7936baf),
        UINT64_C(0xf8474c3bb7cdf024),
        UINT64_C(0x829f3cf387f8795d),
        UINT64_C(0x66e7a46c27f3aa2c),
        UINT64_C(0x1c3fd4a417c62355),
        UINT64_C(0x935745fc4798b8de),
        UINT64_C(0xe98f353477ad31a7),
        UINT64_C(0xa6df411fbfb21ca3),
        UINT64_C(0xdc0731d78f8795da),
        UINT64_C(0x536fa08fdfd90e51),
        UINT64_C(0x29b7d047efec8728),
    };

    /* Copied from redis source code (crc64.c)
     */
    uint64_t crc64(uint64_t crc, const unsigned char *s, uint64_t l)
    {
        uint64_t j;

        for (j = 0; j < l; j++)
        {
            uint8_t byte = s[j];
            crc = crc64_tab[(uint8_t)crc ^ byte] ^ (crc >> 8);
        }
        return crc;
    }

    /* Copied from redis source code (cluster.c)
     *
     * We have 16384 hash slots. The hash slot of a given key is obtained
     * as the least significant 14 bits of the crc16 of the key.
     *
     * However if the key contains the {...} pattern, only the part between
     * { and } is hashed. This may be useful in the future to force certain
     * keys to be in the same node (assuming no resharding is in progress).
     */
    int keyHashSlot(const char *key, size_t keylen)
    {
        size_t s, e; /* start-end indexes of { and } */

        for (s = 0; s < keylen; s++)
            if (key[s] == '{')
                break;

        /* No '{' ? Hash the whole key. This is the base case. */
        if (s == keylen)
            return crc16(key, keylen) & 0x3FFF;

        /* '{' found? Check if we have the corresponding '}'. */
        for (e = s + 1; e < keylen; e++)
            if (key[e] == '}')
                break;

        /* No '}' or nothing betweeen {} ? Hash the whole key. */
        if (e == keylen || e == s + 1)
            return crc16(key, keylen) & 0x3FFF;

        /* If we are here there is both a { and a } on its right. Hash
         * what is in the middle between { and }. */
        return crc16(key + s + 1, e - s - 1) & 0x3FFF; // 0x3FFF == 16383
    }

    // 单机模式，key可以为空
    int get_key_slot(const std::string *key)
    {
        if ((key != NULL) && !key->empty())
        {
            return keyHashSlot(key->c_str(), key->size());
        }
        else
        {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            srandom(tv.tv_usec);
            return random() & 0x3FFF;
        }
    }

    bool keys_crossslots(const std::vector<std::string> &keys)
    {
        if (!keys.empty())
        {
            const int first_key_slot = get_key_slot(&keys[0]);
            for (std::vector<std::string>::size_type i = 1; i < keys.size(); ++i)
            {
                const int cur_key_slot = get_key_slot(&keys[i]);
                if (cur_key_slot != first_key_slot)
                    return true;
            }
        }

        return false;
    }

    void millisleep(int milliseconds)
    {
#if SLEEP_USE_POLL
        // 可配合libco协程库，但可能被中断提前结束而不足milliseconds
        (void)poll(NULL, 0, milliseconds);
#else
        // 无法配合libco协程库
        struct timespec ts = {milliseconds / 1000, (milliseconds % 1000) * 1000000};
        while ((-1 == nanosleep(&ts, &ts)) && (EINTR == errno))
            ;
#endif
    }

    std::string get_formatted_current_datetime(bool with_milliseconds)
    {
        char datetime_buffer[sizeof("YYYY-MM-DD hh:mm:ss/0123456789")];
        struct timeval current;
        gettimeofday(&current, NULL);
        time_t current_seconds = current.tv_sec;

        struct tm result;
        result.tm_isdst = 0;
        localtime_r(&current_seconds, &result);
        if (with_milliseconds)
        {
            snprintf(datetime_buffer, sizeof(datetime_buffer), "%04d-%02d-%02d %02d:%02d:%02d/%u", result.tm_year + 1900, result.tm_mon + 1, result.tm_mday, result.tm_hour, result.tm_min, result.tm_sec, (unsigned int)(current.tv_usec));
        }
        else
        {
            snprintf(datetime_buffer, sizeof(datetime_buffer), "%04d-%02d-%02d %02d:%02d:%02d", result.tm_year + 1900, result.tm_mon + 1, result.tm_mday, result.tm_hour, result.tm_min, result.tm_sec);
        }

#if __cplusplus < 201103L
        return std::string(datetime_buffer);
#else
        return std::move(std::string(datetime_buffer));
#endif // __cplusplus < 201103L
    }

    std::string format_string(const char *format, ...)
    {
        size_t size = 4096;
        std::string buffer(size, '\0');
        char *buffer_p = const_cast<char *>(buffer.data());
        int expected = 0;
        va_list ap;

        while (true)
        {
            va_start(ap, format);
            expected = vsnprintf(buffer_p, size, format, ap);

            va_end(ap);
            if (expected > -1 && expected < static_cast<int>(size))
            {
                break;
            }
            else
            {
                /* The functions snprintf() and vsnprintf() do not write more than size bytes
                 * (including the terminating null byte ('\0')).
                 *
                 * If the output was truncated due to this limit then the return value is the number of characters
                 * (excluding the terminating null byte) which would have been written to the final
                 * string if enough space had been available.
                 *
                 * Thus, a return value of size or more means that the output was truncated.
                 *
                 * The glibc implementation of the functions snprintf() and vsnprintf() conforms to the C99 standard,
                 * that is, behaves as described above, since glibc version 2.1.
                 * Until glibc 2.0.6 they would return -1 when the output was truncated.
                 */

                /* Else try again with more space. */
                if (expected > -1)                            /* glibc 2.1 */
                    size = static_cast<size_t>(expected + 1); /* precisely what is needed */
                else                                          /* glibc 2.0 */
                    size *= 2;                                /* twice the old size */

                buffer.resize(size);
                buffer_p = const_cast<char *>(buffer.data());
            }
        }

        // expected不包含字符串结尾符号，其值等于：strlen(buffer_p)
#if __cplusplus < 201103L
        return std::string(buffer_p, expected > 0 ? expected : 0);
#else
        return std::move(std::string(buffer_p, expected > 0 ? expected : 0));
#endif // __cplusplus < 201103L
    }

    int parse_nodes(std::vector<std::pair<std::string, uint16_t>> *nodes, const std::string &nodes_string)
    {
        std::string::size_type len = 0;
        std::string::size_type pos = 0;
        std::string::size_type comma_pos = 0;

        nodes->clear();
        while (comma_pos != std::string::npos)
        {
            comma_pos = nodes_string.find(',', pos);
            if (comma_pos != std::string::npos)
                len = comma_pos - pos;
            else
                len = nodes_string.size() - comma_pos;

            if (len > 0)
            {
                const std::string &str = nodes_string.substr(pos, len);
                const std::string::size_type colon_pos = str.find(':');
                if (colon_pos != std::string::npos)
                {
                    const std::string &ip_str = str.substr(0, colon_pos);
                    const std::string &port_str = str.substr(colon_pos + 1);
                    nodes->push_back(std::make_pair(ip_str, (uint16_t)atoi(port_str.c_str())));
                }
            }

            pos = comma_pos + 1; // Next node
        }

        return static_cast<int>(nodes->size());
    }

    int split(std::vector<std::string> *tokens, const std::string &source, const std::string &sep, bool skip_sep)
    {
        if (sep.empty())
        {
            tokens->push_back(source);
        }
        else if (!source.empty())
        {
            std::string str = source;
            std::string::size_type pos = str.find(sep);

            while (true)
            {
                const std::string &token = str.substr(0, pos);
                tokens->push_back(token);

                if (std::string::npos == pos)
                {
                    break;
                }
                if (skip_sep)
                {
                    bool end = false;
                    while (0 == strncmp(sep.c_str(), &str[pos + 1], sep.size()))
                    {
                        pos += sep.size();
                        if (pos >= str.size())
                        {
                            end = true;
                            tokens->push_back(std::string(""));
                            break;
                        }
                    }

                    if (end)
                        break;
                }

                str = str.substr(pos + sep.size());
                pos = str.find(sep);
            }
        }

        return static_cast<int>(tokens->size());
    }

    bool parse_node_string(const std::string &node_string, std::string *ip, uint16_t *port)
    {
        // node_string在3.0版本时的格式： 127.0.0.1:1381
        // node_string在4.0版本时的格式：127.0.0.1:1381@11381
        const std::string::size_type colon_pos = node_string.find(':');

        if (colon_pos == std::string::npos)
        {
            return false;
        }
        else
        {
            const std::string &port_str = node_string.substr(colon_pos + 1);
            *port = atoi(port_str.c_str()); // 不管是1381或1381@11381，均可正确得到端口号
            *ip = node_string.substr(0, colon_pos);
            return true;
        }
    }

    void parse_slot_string(const std::string &slot_string, int *start_slot, int *end_slot)
    {
        const std::string::size_type bar_pos = slot_string.find('-');

        if (bar_pos == std::string::npos)
        {
            *start_slot = atoi(slot_string.c_str());
            *end_slot = *start_slot;
        }
        else
        {
            const std::string &end_slot_str = slot_string.substr(bar_pos + 1);
            *end_slot = atoi(end_slot_str.c_str());
            *start_slot = atoi(slot_string.substr(0, bar_pos).c_str());
        }
    }

    // MOVED 9166 10.240.84.140:6379
    bool parse_moved_string(const std::string &moved_string, std::pair<std::string, uint16_t> *node)
    {
        do
        {
            const std::string::size_type space_pos = moved_string.rfind(' ');
            if (space_pos == std::string::npos)
                break;

            const std::string &ip_and_port_string = moved_string.substr(space_pos + 1);
            const std::string::size_type colon_pos = ip_and_port_string.find(':');
            if (colon_pos == std::string::npos)
                break;

            node->first = ip_and_port_string.substr(0, colon_pos);
            node->second = (uint16_t)atoi(ip_and_port_string.substr(colon_pos + 1).c_str());
            if (0 == node->second)
                break;

            return true;
        } while (false);

        return false;
    }

    /* Copied from redis source code (util.c)
     *
     * Return the number of digits of 'v' when converted to string in radix 10.
     * See ll2string() for more information. */
    static uint32_t digits10(uint64_t v)
    {
        if (v < 10)
            return 1;
        if (v < 100)
            return 2;
        if (v < 1000)
            return 3;
        if (v < UINT64_C(1000000000000))
        {
            if (v < UINT64_C(100000000))
            {
                if (v < 1000000)
                {
                    if (v < 10000)
                        return 4;
                    return 5 + (v >= 100000);
                }
                return 7 + (v >= UINT64_C(10000000));
            }
            if (v < UINT64_C(10000000000))
            {
                return 9 + (v >= UINT64_C(1000000000));
            }
            return 11 + (v >= UINT64_C(100000000000));
        }
        return 12 + digits10(v / UINT64_C(1000000000000));
    }

    /* Copied from redis source code (util.c)
     *
     * Convert a long long into a string. Returns the number of
     * characters needed to represent the number.
     * If the buffer is not big enough to store the string, 0 is returned.
     *
     * Based on the following article (that apparently does not provide a
     * novel approach but only publicizes an already used technique):
     *
     * https://www.facebook.com/notes/facebook-engineering/three-optimization-tips-for-c/10151361643253920
     *
     * Modified in order to handle signed integers since the original code was
     * designed for unsigned integers. */
    static int ll2string(char *dst, size_t dstlen, long long svalue)
    {
        static const char digits[201] =
            "0001020304050607080910111213141516171819"
            "2021222324252627282930313233343536373839"
            "4041424344454647484950515253545556575859"
            "6061626364656667686970717273747576777879"
            "8081828384858687888990919293949596979899";
        int negative;
        unsigned long long value;

        /* The main loop works with 64bit unsigned integers for simplicity, so
         * we convert the number here and remember if it is negative. */
        if (svalue < 0)
        {
            if (svalue != LONG_MIN)
            {
                value = -svalue;
            }
            else
            {
                value = ((unsigned long long)LONG_MAX) + 1;
            }
            negative = 1;
        }
        else
        {
            value = svalue;
            negative = 0;
        }

        /* Check length. */
        uint32_t const length = digits10(value) + negative;
        if (length >= dstlen)
            return 0;

        /* Null term. */
        uint32_t next = length;
        dst[next] = '\0';
        next--;
        while (value >= 100)
        {
            int const i = (value % 100) * 2;
            value /= 100;
            dst[next] = digits[i + 1];
            dst[next - 1] = digits[i];
            next -= 2;
        }

        /* Handle last 1-2 digits. */
        if (value < 10)
        {
            dst[next] = '0' + (uint32_t)value;
        }
        else
        {
            int i = (uint32_t)value * 2;
            dst[next] = digits[i + 1];
            dst[next - 1] = digits[i];
        }

        /* Add sign. */
        if (negative)
            dst[0] = '-';
        return length;
    }

    /* Copied from redis source code (util.c)
     *
     * Convert a string into a long long. Returns 1 if the string could be parsed
     * into a (non-overflowing) long long, 0 otherwise. The value will be set to
     * the parsed value when appropriate.
     *
     * Note that this function demands that the string strictly represents
     * a long long: no spaces or other characters before or after the string
     * representing the number are accepted, nor zeroes at the start if not
     * for the string "0" representing the zero number.
     *
     * Because of its strictness, it is safe to use this function to check if
     * you can convert a string into a long long, and obtain back the string
     * from the number without any loss in the string representation. */
    static int string2ll(const char *s, size_t slen, long long *value)
    {
        const char *p = s;
        size_t plen = 0;
        int negative = 0;
        unsigned long long v;

        /* A zero length string is not a valid number. */
        if (plen == slen)
            return 0;

        /* Special case: first and only digit is 0. */
        if (slen == 1 && p[0] == '0')
        {
            if (value != NULL)
                *value = 0;
            return 1;
        }

        /* Handle negative numbers: just set a flag and continue like if it
         * was a positive number. Later convert into negative. */
        if (p[0] == '-')
        {
            negative = 1;
            p++;
            plen++;

            /* Abort on only a negative sign. */
            if (plen == slen)
                return 0;
        }

        /* First digit should be 1-9, otherwise the string should just be 0. */
        if (p[0] >= '1' && p[0] <= '9')
        {
            v = p[0] - '0';
            p++;
            plen++;
        }
        else
        {
            return 0;
        }

        /* Parse all the other digits, checking for overflow at every step. */
        while (plen < slen && p[0] >= '0' && p[0] <= '9')
        {
            if (v > (ULONG_MAX / 10)) /* Overflow. */
                return 0;
            v *= 10;

            if (v > (ULONG_MAX - (p[0] - '0'))) /* Overflow. */
                return 0;
            v += p[0] - '0';

            p++;
            plen++;
        }

        /* Return if not all bytes were used. */
        if (plen < slen)
            return 0;

        /* Convert to negative if needed, and do the final overflow check when
         * converting from unsigned long long to long long. */
        if (negative)
        {
            if (v > ((unsigned long long)(-(LONG_MIN + 1)) + 1)) /* Overflow. */
                return 0;
            if (value != NULL)
                *value = -v;
        }
        else
        {
            if (v > LONG_MAX) /* Overflow. */
                return 0;
            if (value != NULL)
                *value = v;
        }
        return 1;
    }

    std::string int2string(int64_t n)
    {
        std::string str(sizeof("18446744073709551615") + 1, '\0');
        char *str_p = const_cast<char *>(str.c_str());
        const int len = ll2string(str_p, str.size(), n);
        str.resize(len);

#if __cplusplus < 201103L
        return str;
#else
        return std::move(str);
#endif // __cplusplus < 201103L
    }

    std::string int2string(int32_t n)
    {
        return int2string(static_cast<int64_t>(n));
    }

    std::string int2string(int16_t n)
    {
        return int2string(static_cast<int64_t>(n));
    }

    std::string int2string(uint64_t n)
    {
        return int2string(static_cast<int64_t>(n));
    }

    std::string int2string(uint32_t n)
    {
        return int2string(static_cast<int64_t>(n));
    }

    std::string int2string(uint16_t n)
    {
        return int2string(static_cast<uint64_t>(n));
    }

    bool string2int(const char *s, size_t len, int64_t *val, int64_t errval)
    {
        long long llval = 0;
        if (1 == string2ll(s, len, &llval))
        {
            *val = static_cast<int64_t>(llval);
            return true;
        }
        else
        {
            *val = errval;
            return false;
        }
    }

    bool string2int(const char *s, size_t len, int32_t *val, int32_t errval)
    {
        const int64_t errval64 = errval;
        int64_t val64 = 0;

        if (string2int(s, len, &val64, errval64))
        {
            *val = static_cast<int32_t>(val64);
            return true;
        }
        else
        {
            *val = errval;
            return false;
        }
    }

    uint64_t get_random_number(uint64_t base)
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        srandom(tv.tv_usec);
        return base + random();
    }

    
}
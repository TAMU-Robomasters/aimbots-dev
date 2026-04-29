    #pragma once

    #include "modm/platform/spi/spi_master_2.hpp"
    #include "utils/tools/common_types.hpp"
    #include <tap/algorithms/wrapped_float.hpp>
    #include <cstdint>

    namespace src {
    class Drivers;
    }

    namespace src::Informants {

    class RevEncoder
    {
    public:
        enum class EncoderID : uint8_t
        {
            REV_ENCODER_1 = 0b000,
            REV_ENCODER_2 = 0b001,
            REV_ENCODER_3 = 0b010,
            REV_ENCODER_4 = 0b011,
            REV_ENCODER_5 = 0b100
        };

        enum class SpiReadState
        {
            SelectEncoder,
            ReadEncoder
        };

        explicit RevEncoder(src::Drivers* drivers);

        void initialize();
        void selectEncoder(EncoderID encoder);
        void execute();

        uint16_t getData() const;
        float getVelocity() const;
        int64_t getUnwrappedPosition() const;

    private:
        void readData();
        void readAll();
        void revEncoderVelocity();

        src::Drivers* drivers;

        uint16_t data = 0;
        static constexpr uint8_t NUM_ENCODERS = 5;
        uint16_t allData[NUM_ENCODERS] = {};
        float velocity = 0.0f;

        int64_t unwrappedPosition = 0;
        uint16_t lastRawCount = 0;
        bool unwrappedInitialized = false;

        tap::algorithms::WrappedFloat lastAngle{0.0f, -M_PI, M_PI};
        uint32_t lastTimeMs = 0;
        uint32_t currentTimeMs = 0;
        uint32_t dtMs = 0;

        float filteredAngle = 0.0f;
        bool angleFilterInitialized = false;

        float filteredVelocity = 0.0f;
        bool velocityFilterInitialized = false;

        static constexpr float kPositionAlpha = 0.10f;
        static constexpr float kVelocityAlpha = 0.20f;

        MilliTimeout startupThreshold;
        SpiReadState readState = SpiReadState::SelectEncoder;
        EncoderID currentEncoder = EncoderID::REV_ENCODER_1;
    };

    }  // namespace src::Informants
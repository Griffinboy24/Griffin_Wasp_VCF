#pragma once 
#include <JuceHeader.h> 
#include <cmath>

namespace project {

    using namespace juce;
    using namespace hise;
    using namespace scriptnode;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    template <int NV>
    struct Griffin_Wasp : public data::base
    {
        SNEX_NODE(Griffin_Wasp);

        struct MetadataClass {
            SN_NODE_ID("Griffin_Wasp");
        };

        static constexpr bool isModNode() { return false; }
        static constexpr bool isPolyphonic() { return NV > 1; }
        static constexpr bool hasTail() { return false; }
        static constexpr bool isSuspendedOnSilence() { return false; }
        static constexpr int getFixChannelAmount() { return 2; }

        static constexpr int NumTables = 0;
        static constexpr int NumSliderPacks = 0;
        static constexpr int NumAudioFiles = 0;
        static constexpr int NumFilters = 0;
        static constexpr int NumDisplayBuffers = 0;

        int filterType = 0;

        struct BiquadState {
            float x1 = 0.0f, x2 = 0.0f;
            float y1 = 0.0f, y2 = 0.0f;
        };

        BiquadState lpState[2];
        BiquadState bpState[2];
        BiquadState hpState[2];

        struct Coefficients {
            float b0, b1, b2;
            float a1, a2;
        };

        Coefficients lpCoeff;
        Coefficients bpCoeff;
        Coefficients hpCoeff;

        // Component values 
        const float R1 = 27000.0f;
        const float R2 = 27000.0f;
        const float R3 = 27000.0f;
        const float R4 = 27000.0f;
        const float R13 = 1000000.0f;
        const float R14 = 1000.0f;
        const float R15 = 100000.0f;
        const float C2 = 100e-12f;
        const float C7 = 0.22e-6f;

      
        float R_res() const { return ((R13 + R15) * R14) / (R13 + R15 + R14); }

   
        void updateCoefficients(float sampleRate, float cutoff, float resonance)
        {
            float wc = 2.0f * float(M_PI) * cutoff;
            float i_bias = wc / 9.855e8f; // OTA bias current

            float rho = resonance;
            float Rres = R_res();
            float Rres_rho = (rho * Rres) + R14;

            // Resonance network coefficients (from design documents)
            float b1 = R3 * (1.0f - rho) * Rres * (R13 + R15) * C7;
            float b0 = R3 * (Rres + R13 + R14 + R15);
            float a1 = ((Rres_rho * (1.0f - rho) * Rres) + ((Rres + R13 + R15) * R4)) * (R13 + R15) * C7;
            float a0 = ((Rres_rho + R4) * (Rres + R13 + R14 + R15)) - (Rres_rho * Rres_rho);

         
            float H1_limit = b1 / a1;
            float Q = 1.0f / (H1_limit + R2 * C2 * wc);

            float T = 1.0f / sampleRate;
            float K = std::tan(wc * T / 2.0f);
            float A = 1.0f + K / Q + K * K;

            // Lowpass coefficients (standard bilinear transform for 2nd order LP)
            lpCoeff.b0 = (K * K) / A;
            lpCoeff.b1 = 2.0f * (K * K) / A;
            lpCoeff.b2 = (K * K) / A;
            lpCoeff.a1 = 2.0f * (K * K - 1.0f) / A;
            lpCoeff.a2 = (1.0f - K / Q + K * K) / A;

     
            bpCoeff.b0 = 2.0f * (K / Q) / A;
            bpCoeff.b1 = 0.0f;
            bpCoeff.b2 = -2.0f * (K / Q) / A;
            bpCoeff.a1 = lpCoeff.a1;
            bpCoeff.a2 = lpCoeff.a2;

            // Highpass coefficients
            hpCoeff.b0 = 1.0f / A;
            hpCoeff.b1 = -2.0f / A;
            hpCoeff.b2 = 1.0f / A;
            hpCoeff.a1 = lpCoeff.a1;
            hpCoeff.a2 = lpCoeff.a2;
        }

        // Reset all biquad state variables.
        void resetStates()
        {
            for (int ch = 0; ch < 2; ++ch)
            {
                lpState[ch] = { 0.0f, 0.0f, 0.0f, 0.0f };
                bpState[ch] = { 0.0f, 0.0f, 0.0f, 0.0f };
                hpState[ch] = { 0.0f, 0.0f, 0.0f, 0.0f };
            }
        }

        // Per-channel audio effect parameters.
        class AudioEffect
        {
        public:
            AudioEffect() {}
            float cutoff = 5000.0f;
            float resonance = 0.5f;
            float sampleRate = 44100.0f;
        };

        AudioEffect leftChannelEffect;
        AudioEffect rightChannelEffect;

        // Prepare the filter with the given sample rate.
        void prepare(PrepareSpecs specs)
        {
            float sr = specs.sampleRate;
            leftChannelEffect.sampleRate = sr;
            rightChannelEffect.sampleRate = sr;
            resetStates();
            updateCoefficients(sr, 5000.0f, 0.5f);
        }

        void reset() {}

        // Process an audio block.
        template <typename ProcessDataType>
        void process(ProcessDataType& data)
        {
            auto& fixData = data.template as<ProcessData<getFixChannelAmount()>>();
            auto audioBlock = fixData.toAudioBlock();
            int blockSize = data.getNumSamples();

            float* leftChannelData = audioBlock.getChannelPointer(0);
            float* rightChannelData = audioBlock.getChannelPointer(1);

            for (int i = 0; i < blockSize; ++i)
            {
                float inL = leftChannelData[i];
                float inR = rightChannelData[i];

                float outLP_L = processBiquad(inL, lpCoeff, lpState[0]);
                float outBP_L = processBiquad(inL, bpCoeff, bpState[0]);
                float outHP_L = processBiquad(inL, hpCoeff, hpState[0]);

                float outLP_R = processBiquad(inR, lpCoeff, lpState[1]);
                float outBP_R = processBiquad(inR, bpCoeff, bpState[1]);
                float outHP_R = processBiquad(inR, hpCoeff, hpState[1]);

                float mainOutL = (filterType == 0) ? outLP_L : (filterType == 1 ? outBP_L : outHP_L);
                float mainOutR = (filterType == 0) ? outLP_R : (filterType == 1 ? outBP_R : outHP_R);

                leftChannelData[i] = mainOutL;
                rightChannelData[i] = mainOutR;
            }
        }

        // Process a single sample through the biquad difference equation.
        float processBiquad(float x, const Coefficients& coeff, BiquadState& state)
        {
            float y = coeff.b0 * x + coeff.b1 * state.x1 + coeff.b2 * state.x2 -
                coeff.a1 * state.y1 - coeff.a2 * state.y2;
            state.x2 = state.x1;
            state.x1 = x;
            state.y2 = state.y1;
            state.y1 = y;
            return y;
        }

        // Parameter handling.
        template <int P>
        void setParameter(double v)
        {
            if (P == 0)
            {
                leftChannelEffect.cutoff = static_cast<float>(v);
                rightChannelEffect.cutoff = static_cast<float>(v);
                updateCoefficients(leftChannelEffect.sampleRate, static_cast<float>(v), leftChannelEffect.resonance);
            }
            else if (P == 1)
            {
                leftChannelEffect.resonance = static_cast<float>(v);
                rightChannelEffect.resonance = static_cast<float>(v);
                updateCoefficients(leftChannelEffect.sampleRate, leftChannelEffect.cutoff, static_cast<float>(v));
            }
            else if (P == 2)
            {
                filterType = static_cast<int>(v);
            }
        }

        // Create GUI parameters.
        void createParameters(ParameterDataList& data)
        {
            {
                parameter::data p("Cutoff", { 20.0, 20000.0, 0.01 });
                registerCallback<0>(p);
                p.setDefaultValue(5000.0);
                data.add(std::move(p));
            }
            {
                parameter::data p("Resonance", { 0.0, 1.0, 0.01 });
                registerCallback<1>(p);
                p.setDefaultValue(0.5);
                data.add(std::move(p));
            }
            {
                parameter::data p("FilterType", { 0, 2, 1 });
                registerCallback<2>(p);
                p.setDefaultValue(0);
                data.add(std::move(p));
            }
        }

        void setExternalData(const ExternalData& ed, int index) {}
        void handleHiseEvent(HiseEvent& e) {}
        template <typename FrameDataType>
        void processFrame(FrameDataType& data) {}
    };

	} // namespace project

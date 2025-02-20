#pragma once
#include <JuceHeader.h>
#include <cmath>

namespace project
{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    using namespace juce;
    using namespace hise;
    using namespace scriptnode;

    namespace FunctionsClasses {
        // Additional helper functions or classes can be defined here if needed.
    }

    template <int NV>
    struct Griffin_Wasp : public data::base
    {
        SNEX_NODE(Griffin_Wasp);

        struct MetadataClass
        {
            SN_NODE_ID("Griffin_Wasp");
        };

        //==============================================================================
        // Node Properties 
        //==============================================================================

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

        //==============================================================================
        // Audio Effect Class (Linear WASP Filter DSP Implementation)
        //==============================================================================
        class AudioEffect
        {
        public:
            // Constructor sets initial cutoff (Hz) and resonance (0-1)
            AudioEffect(float initCutoff = 5000.0f, float initResonance = 0.5f)
                : cutoff(initCutoff), resonance(initResonance)
            {
                sampleRate = 44100.0;
                resetState();
                recalcCoeffs();
            }

            // Initialize filter (called on prepare)
            void prepare(double sr)
            {
                sampleRate = sr;
                resetState();
                recalcCoeffs();
            }

            // Process a block of samples using the difference equation
            void process(float* samples, int numSamples)
            {
                for (int i = 0; i < numSamples; ++i)
                {
                    float x = samples[i];
                    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
                    samples[i] = y;
                    // Update delay elements
                    x2 = x1;
                    x1 = x;
                    y2 = y1;
                    y1 = y;
                }
            }

            // Update cutoff parameter and recalc coefficients
            void updateCutoff(float newCutoff)
            {
                cutoff = newCutoff;
                recalcCoeffs();
            }

            // Update resonance parameter and recalc coefficients
            void updateResonance(float newRes)
            {
                resonance = newRes;
                recalcCoeffs();
            }

        private:
            double sampleRate;
            float cutoff;     // Filter cutoff in Hz
            float resonance;  // Resonance control (0 to 1)

            // Difference equation coefficients
            float a1 = 0.0f, a2 = 0.0f;
            float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;

            // State variables for delay elements
            float x1 = 0.0f, x2 = 0.0f;
            float y1 = 0.0f, y2 = 0.0f;

            // Recalculate filter coefficients using the bilinear transform
            void recalcCoeffs()
            {
                // Analog cutoff in rad/s
                float omega = 2.0f * M_PI * cutoff;
                // Offset from OTA integrator branch (R2=27k, C2=100pF)
                float offset = 27000.0f * 100e-12f * omega; // ~1.696e-5 * cutoff
                // Calculate Q from resonance parameter and offset
                float Q = 1.0f / (resonance + offset);

                // Pre-warp using tan(pi*Cutoff/sampleRate)
                float K = std::tan(M_PI * cutoff / static_cast<float>(sampleRate));
                float K2 = K * K;
                float norm = 1.0f / (1.0f + K / Q + K2);

                // Coefficients derived from analog prototype:
                // H(s) = omega^2/(s^2 + (omega/Q)*s + omega^2)
                b0 = K2 * norm;
                b1 = 2.0f * b0;
                b2 = b0;
                a1 = 2.0f * (K2 - 1.0f) * norm;
                a2 = (1.0f - K / Q + K2) * norm;
            }

            void resetState()
            {
                x1 = x2 = y1 = y2 = 0.0f;
            }
        };

        //==============================================================================
        // Main Processing Functions
        //==============================================================================

        void prepare(PrepareSpecs specs)
        {
            float sr = specs.sampleRate;
            leftChannelEffect.prepare(sr);
            rightChannelEffect.prepare(sr);
        }

        void reset() {}

        template <typename ProcessDataType>
        void process(ProcessDataType& data)
        {
            auto& fixData = data.template as<ProcessData<getFixChannelAmount()>>();
            auto audioBlock = fixData.toAudioBlock();

            auto* leftChannelData = audioBlock.getChannelPointer(0);
            auto* rightChannelData = audioBlock.getChannelPointer(1);

            int blockSize = data.getNumSamples();

            leftChannelEffect.process(leftChannelData, blockSize);
            rightChannelEffect.process(rightChannelData, blockSize);
        }

        //==============================================================================
        // Parameter Handling
        //==============================================================================

        // Parameter index 0: Cutoff frequency; index 1: Resonance
        template <int P>
        void setParameter(double v)
        {
            if (P == 0)
            {
                leftChannelEffect.updateCutoff(static_cast<float>(v));
                rightChannelEffect.updateCutoff(static_cast<float>(v));
            }
            else if (P == 1)
            {
                leftChannelEffect.updateResonance(static_cast<float>(v));
                rightChannelEffect.updateResonance(static_cast<float>(v));
            }
        }

        // Create GUI parameters
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
        }

        //==============================================================================
        // External Data Handling
        //==============================================================================

        void setExternalData(const ExternalData& ed, int index)
        {
            /*
            // Example external data handling:
            ExternalData data = ed;
            float sampleRate = data.sampleRate;
            float numChannels = data.numChannels;
            ed.referBlockTo(externalBuffer[0], 0);
            ed.referBlockTo(externalBuffer[1], 1);
            */
        }

        //==============================================================================
        // Event Handling (Modify as needed for MIDI or other events)
        //==============================================================================

        void handleHiseEvent(HiseEvent& e)
        {
            /*
            if (e.isNoteOn())
            {
                float note = e.getNoteNumber();
            }
            */
        }

        //==============================================================================
        // Frame Processing (Required by compiler, but not used in block processing)
        //==============================================================================

        template <typename FrameDataType>
        void processFrame(FrameDataType& data) {}

    private:
        AudioEffect leftChannelEffect;
        AudioEffect rightChannelEffect;
    };

}

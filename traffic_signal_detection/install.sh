#!/bin/bash
# Installation script for Traffic Signal Detection System

echo "======================================================================"
echo "🚦 Traffic Signal Detection - Installation Script"
echo "======================================================================"
echo ""

# Check Python version
echo "Checking Python version..."
python_version=$(python3 --version 2>&1)
echo "✅ Found: $python_version"
echo ""

# Install dependencies
echo "Installing required Python packages..."
echo "This will install:"
echo "  - opencv-python (Computer Vision)"
echo "  - numpy (Numerical Computing)"
echo ""

pip3 install -r requirements.txt

if [ $? -eq 0 ]; then
    echo ""
    echo "======================================================================"
    echo "✅ Installation Complete!"
    echo "======================================================================"
    echo ""
    echo "📋 Next Steps:"
    echo ""
    echo "1. Grant camera permissions (macOS only):"
    echo "   System Settings → Privacy & Security → Camera → Enable Terminal"
    echo ""
    echo "2. Run the detector:"
    echo "   python3 traffic_signal_detector.py"
    echo ""
    echo "3. Press 'q' to quit the detector"
    echo ""
    echo "📖 Documentation:"
    echo "   - Quick Start: GETTING_STARTED.md"
    echo "   - Full Guide: README.md"
    echo "   - Technical: IMPROVEMENTS.md"
    echo ""
    echo "======================================================================"
else
    echo ""
    echo "❌ Installation failed. Please check the error messages above."
    echo ""
    echo "Try manual installation:"
    echo "  pip3 install opencv-python>=4.8.0"
    echo "  pip3 install numpy>=1.24.0"
fi

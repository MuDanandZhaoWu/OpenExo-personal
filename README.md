# Teensy Nano Board

![Diagram](/Documentation/Figures/Code_Structure.png)

This code uses two microcontrollers to control wearable robots. In order to
build the and flash the codebase check [here](Documentation/BUILD_AND_FLASH.md) and [here](https://github.com/naubiomech/TeensyNanoExoCode/blob/main/Documentation/README.md#how-to-deploy). If you would like to
contribute check [here](CONTRIBUTING.md) for more details. 
 
Check out the [Documentation Folder](/Documentation) for more details and C++ info.

Documentation on the software can be found on our [read the docs page](https://theopenexo.readthedocs.io/en/latest/index.html).

Documentation on the hardware can be found on our [wiki](https://youneedawiki.com/app/page/14AIGjap02Wv8jPJxyezvfYJYFVIJIoO1?p=14AIGjap02Wv8jPJxyezvfYJYFVIJIoO1).

Video walkthroughs on getting started and indepth looks at different aspects of the system can be found on the OpenExo [YouTube page](https://www.youtube.com/@TheOpenExo).

## Repository contents

Firmware and GUI source, SD-card configuration templates, documentation sources,
and the vendored `Libraries/` dependencies are versioned. Keep the libraries:
some contain project-specific changes documented in [Libraries/README.md](Libraries/README.md).

This fork retains the Doxygen output (`html/`, `latex/`) and the existing Sphinx
output (`Documentation/ReadTheDocs/source/_build/`) to follow the
[upstream OpenExo repository](https://github.com/naubiomech/OpenExo) layout.

Trial recordings, saved device addresses, and GUI preferences in
`Python_GUI/Saved_Data/` are local files and are not versioned. The GUI creates
the directory as needed. Back up experiment data separately from this repository.

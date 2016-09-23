# Camera folder

Screenshots from motion recording are saved here, with each folder
corresponding to a camera name. To assemble all screenshots from camera
``blah`` (folder ``blah/``), do:

```bash
./assemble.zsh blah
```

## Split-view video

To generate the split-view video accompanying the paper, run the motion editor
in recording mode to populate the camera folders ``camera1/`` and ``camera2/``,
then do:

```bash
./generate_split.zsh
```

Assembled screenshots will be stored in ``split/``. You can finally call
``assemble.zsh`` to make the video.

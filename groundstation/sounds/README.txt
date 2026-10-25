Own recordings for the voice callouts (optional)
================================================

A WAV file here replaces the computer voice for one phrase:

    sounds\el\<key>.wav     Greek
    sounds\en\<key>.wav     English

<key> is the phrase's key in lang\el.txt / lang\en.txt, for example

    sounds\el\say.alarm.compass_lost.wav
    sounds\en\say.mode.auto.wav

Any WAV works (mono or stereo, 8 or 16 bit, any sample rate). A callout
made of several phrases (traffic: say.traffic + say.clock_2 + say.dist_m_800)
uses recordings only when every one of its phrases has one.
The Sound tab of the captain screen plays every callout to check it.

# Notes courtesy of https://gist.github.com/i-Robi/8684800?permalink_comment_id=2307473#gistcomment-2307473
# Note that A# = Bb, C# = Db, D# = Eb, F# = Gb, G# = Ab
notes = {
    "C": [16.35, 32.70, 65.41, 130.81, 261.63, 523.25, 1046.50, 2093.00, 4186.01],
   "Db":   [17.32, 34.65, 69.30, 138.59, 277.18, 554.37, 1108.73, 2217.46, 4434.92],
    "D":   [18.35, 36.71, 73.42, 146.83, 293.66, 587.33, 1174.66, 2349.32, 4698.64],
   "Eb":   [19.45, 38.89, 77.78, 155.56, 311.13, 622.25, 1244.51, 2489.02, 4978.03],
    "E":   [20.60, 41.20, 82.41, 164.81, 329.63, 659.26, 1318.51, 2637.02],
    "F":   [21.83, 43.65, 87.31, 174.61, 349.23, 698.46, 1396.91, 2793.83],
   "Gb":   [23.12, 46.25, 92.50, 185.00, 369.99, 739.99, 1479.98, 2959.96],
    "G":   [24.50, 49.00, 98.00, 196.00, 392.00, 783.99, 1567.98, 3135.96],
   "Ab":   [25.96, 51.91, 103.83, 207.65, 415.30, 830.61, 1661.22, 3322.44],
    "A":   [27.50, 55.00, 110.00, 220.00, 440.00, 880.00, 1760.00, 3520.00],
   "Bb":   [29.14, 58.27, 116.54, 233.08, 466.16, 932.33, 1864.66, 3729.31],
    "B":   [30.87, 61.74, 123.47, 246.94, 493.88, 987.77, 1975.53, 3951.07]
 }

# When the number is not specified, it is a middle note (which is the 4th entry)
# So these are to make it easier
A = notes["A"][4]
B = notes["B"][4]
C = notes["C"][4]
D = notes["D"][4]
E = notes["E"][4]
F = notes["F"][4]
G = notes["G"][4]

As = notes["Bb"][4]
Cs = notes["Db"][4]
Ds = notes["Eb"][4]
Fs = notes["Gb"][4]
Gs = notes["Ab"][4]

class Song:
    def __init__(self, name, tempo, notes, timeBetween):
        self.name = name
        self.tempo = tempo
        self.beat = 60.0 / tempo
        self.notes = notes
        self.timeBetween = timeBetween
    
    # Setting the note durations is done after creating the instance of the class so that beat is defined.
    def setNoteDuration(self, noteDurations):
        self.noteDurations = noteDurations

twinkleTwinkle = Song("Twinkle Twinkle Little Star", 200, [C, C, G, G, A, A, G, 
            F, F, E, E, D, D, C,
            G, G, F, F, E, E, D,
            G, G, F, F, E, E, D,
            C, C, G, G, A, A, G, 
            F, F, E, E, D, D, C], 0.25) 
twinkleTwinkle.setNoteDuration(([1*twinkleTwinkle.beat] * 6 + [2*twinkleTwinkle.beat]) * 6)

immortals = Song("Immortals", 400, 
            [Cs, A, A, A, A, A, Fs, Fs,
             A, A, Fs, Fs,
             A, A, E, Cs, A, Gs],
             0.25)
immortals.setNoteDuration(([1*immortals.beat] * 5 + [2*immortals.beat] * 13))

shakeItOff = Song("Shake It Off", 
            350,
            [A, notes["C"][5], notes["D"][5], notes["D"][5], notes["D"][5], notes["E"][5],
            notes["C"][5], A, G, E, C,
            A, notes["C"][5], notes["D"][5], notes["D"][5], notes["D"][5], notes["E"][5],
            notes["C"][5], A, G, E, C,
            notes["C"][5], notes["C"][5], notes["D"][5], notes["D"][5], notes["D"][5], notes["E"][5],
            notes["C"][5], A, G, E, C,
            0, notes["C"][5], notes["D"][5], notes["E"][5], notes["C"][5], 0, 0, notes["C"][5], notes["D"][5], notes["E"][5], notes["C"][5]],
            0.15)
shakeItOff.setNoteDuration( ([1*shakeItOff.beat] * 6 + [2*shakeItOff.beat] * 5) * 3 + [1*shakeItOff.beat] * 11)

balalala = Song("balalala", 800, [F, Gs, B, notes["Db"][5], notes["Eb"][5], notes["Db"][5], notes["B"][3]], 0)
balalala.setNoteDuration(([4*balalala.beat] + [1*balalala.beat]) * 4)
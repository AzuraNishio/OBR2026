class Pessoa:
    def __init__(self, name: str, age: int, species = "Human"):
        self.name = name
        self.age = age
        self.species = species

    def introduce_myself(self):
        print(f"Hi! I'm {self.name}! I am {self.age} years old! I'm a {self.species}!")

    def age_by(self, delta):
        self.age += delta


Haig = Pessoa("Haig", 20, species= "dog")
Azura = Pessoa("Azura", 17, species="Cat")

Haig.introduce_myself()
Azura.introduce_myself()

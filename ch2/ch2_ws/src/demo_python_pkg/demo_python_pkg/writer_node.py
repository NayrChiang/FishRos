from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode):
    def __init__(self, name: str, age: int, book: str) -> None:
        super().__init__(name, age)
        self.book = book


def main():
    node = WriterNode('Katy', 25, 'God of War')
    node.eat('Sea Urchin')
    
    

import rclpy
from rclpy.node import Node

class PersonNode(Node):
    def __init__(self, node_name:str, name: str, age: int) -> None:
        print("PersonNode __init__ method has been used.")
        super().__init__(node_name)
        self.name = name
        self.age = age

    def eat(self, food: str):
        """
        Method: Eat 
        :food - name of food
        """
        # print(f"{self.name}, age {self.age}, loves to eat {food}")
        self.get_logger().info(f"{self.name}, age {self.age}, loves to eat {food}")


def main():
    rclpy.init()
    node = PersonNode('katyn', 'Katy', 24)
    node.eat('Sea Urchin')
    rclpy.spin(node)
    rclpy.shutdown()


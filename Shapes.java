interface Shape {
	final double PI = 3.14;
	void draw(); // ������ �׸��� �߻� �޼ҵ�
	double getArea(); // ������ ������ �����ϴ� �߻� �޼ҵ�
	default public void redraw() { // ����Ʈ �޼ҵ�
		System.out.print("--- �ٽ� �׸��ϴ�. ");
		draw();	
	}
}

class Circle implements Shape {
	private int radius; // ������
	
	public Circle(int radius) { 
		this.radius = radius;
	}
	
	@Override
	public void draw() {
		System.out.println("�������� " + radius + "�� ���Դϴ�.");
	}
	
	@Override
	public double getArea() { 
		return PI*radius*radius;
	}
}

class Oval implements Shape {
	private int a;  
	private int b;
	
	public Oval(int a, int b) { 
		this.a = a;
		this.b = b;
	}
	
	@Override
	public void draw() {
		System.out.println(a+"x"+b+"�� �����ϴ� Ÿ���Դϴ�.");
	}
	
	@Override
	public double getArea() { 
		return PI*a*b;
	}
}

class Rect implements Shape {
	private int c;
	private int d;
	
	public Rect(int c, int d) { 
		this.c = c;
		this.d = d;
	}
	
	@Override
	public void draw() {
		System.out.println(c+"x"+d+"ũ���� �簢�� �Դϴ�.");
	}
	
	@Override
	public double getArea() { 
		return c*d;
	}
}





public class Shapes {
	static public void main(String [] args) {
		Shape [] list = new Shape[3]; // Shape�� ��ӹ��� Ŭ���� ��ü�� ���۷��� �迭
		list[0] = new Circle(10); // �������� 10�� �� ��ü
		
		/*list[0].redraw();
		System.out.println("������ " + list[0].getArea());*/
		 
	     //(�׽�Ʈ��)
		   list[1] = new Oval(20, 30); // 20x30 �簢���� �����ϴ� Ÿ��
		   list[2] = new Rect(10, 40); // 10x40 ũ���� �簢��

		   for(int i=0; i<list.length; i++) list[i].redraw();
		   for(int i=0; i<list.length; i++) System.out.println("������ " + list[i].getArea());
		
	}
}

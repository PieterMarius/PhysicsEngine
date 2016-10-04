using OpenTK;

namespace TestPhysics
{
	class MainClass
	{
		public static void Main (string[] args)
		{
			
			using (TestWindow test = new TestWindow ()) 
			{
				

				//test.VSync = VSyncMode.Adaptive;
				test.Run(0.0, 0.0);
				
			}
		}
	}
}

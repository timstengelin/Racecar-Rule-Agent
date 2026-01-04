import sys, importlib, argparse

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description ='Simulate the Mountain Car.')
	parser.add_argument('agent_file')
	parser.add_argument('data_file')
	parser.add_argument('num_steps', type=int)
	args = parser.parse_args()

	try:
		moduleName = args.agent_file.split('.')[0]
		agentModule = importlib.import_module(moduleName)
	except ModuleNotFoundError as e:
		print(f"Error: Module '{moduleName}' not found. Details: {e}")
		sys.exit()
	except Exception as e:
		print(f"An unexpected error occurred during import: {e}")
		import traceback
		traceback.print_exc()
		sys.exit()
		
	agent = agentModule.Agent()
	try:
		agent.load(args.data_file)
	except:
		print('Could not load data, starting with zeros')

	agent.train(args.num_steps)
	
	agent.save(args.data_file)
